
#include <errno.h>
#include <pthread.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/epoll.h>
#include <math.h>
#include <time.h>
#include <semaphore.h>
#include <signal.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h> 

//-----------------------------------------------------------------------------------------------------------------------
#define  LOG_TAG  "odroid_gps"

#include <cutils/log.h>
#include <cutils/sockets.h> 
#include <cutils/properties.h>
//#include <hardware_legacy/gps.h>
#include <hardware/gps.h>

#define  GPS_DEBUG 1 

#define  LOGI ALOGI
#define  LOGD ALOGD
#define  LOGE ALOGE

#if GPS_DEBUG
#  define  D(...)   ALOGD(__VA_ARGS__)
#else
#  define  D(...)   ((void)0)
#endif

#define  GPS_CHANNEL_NAME  "ttyS4"
//#define  GPS_CHANNEL_NAME  "ttyACM0"
#define  GPS_GPIO_DEV_NAME      "/sys/devices/platform/misc-control/gps_reset"   

#define GPS_GPIO_INCLUDE     1


static time_t utc_mktime(struct tm *_tm);

/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       N M E A   T O K E N I Z E R                     *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/

typedef struct {
    const char*  p;
    const char*  end;
} Token;

#define  MAX_NMEA_TOKENS  16

typedef struct {
    int     count;
    Token   tokens[ MAX_NMEA_TOKENS ];
} NmeaTokenizer;

static int
nmea_tokenizer_init( NmeaTokenizer*  t, const char*  p, const char*  end )
{
    int    count = 0;
    char*  q;

    // the initial '$' is optional
    if (p < end && p[0] == '$')
        p += 1;

    // remove trailing newline
    if (end > p && end[-1] == '\n') {
        end -= 1;
        if (end > p && end[-1] == '\r')
            end -= 1;
    }

    // get rid of checksum at the end of the sentecne
    if (end >= p+3 && end[-3] == '*') {
        end -= 3;
    }

    while (p < end) {
        q = memchr(p, ',', end-p);
        if (q == NULL)
            q = end;
        if(count >= MAX_NMEA_TOKENS)
            break;
        t->tokens[count].p = p;
        t->tokens[count++].end = q;
        p = q + 1;
    }

    t->count = count;
    return count;
}

static Token
nmea_tokenizer_get( NmeaTokenizer*  t, int  index )
{
    Token  tok;
    static const char*  dummy = "";

    if (index < 0 || index >= t->count) {
        tok.p = tok.end = dummy;
    } else
        tok = t->tokens[index];

    return tok;
}


static int
str2int( const char*  p, const char*  end )
{
    int   result = 0;
    int   len    = end - p;

    for ( ; len > 0; len--, p++ )
    {
        int  c;

        if (p >= end)
            goto Fail;

        c = *p - '0';
        if ((unsigned)c >= 10)
            goto Fail;

        result = result*10 + c;
    }
    return  result;

Fail:
    return -1;
}

static double
str2float( const char*  p, const char*  end )
{
    int   result = 0;
    int   len    = end - p;
    char  temp[16];

    if (len >= (int)sizeof(temp))
        return 0.;

    memcpy( temp, p, len );
    temp[len] = 0;
    return strtod( temp, NULL );
}

/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       N M E A   P A R S E R                           *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/

#define  NMEA_MAX_SIZE  83

typedef struct {
    int     pos;
    int     overflow;
    int     utc_year;
    int     utc_mon;
    int     utc_day;
    GpsLocation  fix;
    gps_location_callback  location_cb;
    GpsSvStatus status;
    gps_sv_status_callback  sv_status_cb;
    char    in[ NMEA_MAX_SIZE+1 ];
} NmeaReader;



static void
nmea_reader_init( NmeaReader*  r )
{
    memset( r, 0, sizeof(*r) );

    r->pos      = 0;
    r->overflow = 0;
    r->utc_year = -1;
    r->utc_mon  = -1;
    r->utc_day  = -1;

}


static void
nmea_reader_set_callback( NmeaReader*  r, gps_location_callback  location_cb, gps_sv_status_callback status_cb)
{
    r->location_cb = location_cb;
    r->sv_status_cb = status_cb;
    if (location_cb != NULL && r->fix.flags != 0) {
        D("%s: sending latest fix to new location_cb", __FUNCTION__);
        r->location_cb( &r->fix );
        r->fix.flags = 0;
    }
}


static int
nmea_reader_update_time( NmeaReader*  r, Token  tok )
{
    int        hour, minute;
    double     seconds;
    struct tm  tm;
    time_t     fix_time;

    if (tok.p + 6 > tok.end)
        return -1;

    if (r->utc_year <= 2000) {
		r->utc_year = 1970;
		r->utc_mon = 1;
		r->utc_day = 1;
    }

    hour    = str2int(tok.p,   tok.p+2);
    minute  = str2int(tok.p+2, tok.p+4);
    seconds = str2float(tok.p+4, tok.end);

    tm.tm_hour = hour;
    tm.tm_min  = minute;
    tm.tm_sec  = (int) seconds;
    tm.tm_year = r->utc_year - 1900;
    tm.tm_mon  = r->utc_mon - 1;
    tm.tm_mday = r->utc_day;

    fix_time = utc_mktime( &tm );
    r->fix.timestamp = (long long)fix_time * 1000;
    return 0;
}

static int
nmea_reader_update_date( NmeaReader*  r, Token  date, Token  time )
{
    Token  tok = date;
    int    day, mon, year;

    if (tok.p + 6 != tok.end) {
        D("date not properly formatted: '%.*s'", tok.end-tok.p, tok.p);
        return -1;
    }
    day  = str2int(tok.p, tok.p+2);
    mon  = str2int(tok.p+2, tok.p+4);
    year = str2int(tok.p+4, tok.p+6);

    if ((day|mon|year) < 0) {
        D("date not properly formatted: '%.*s'", tok.end-tok.p, tok.p);
        return -1;
    }

    r->utc_year  = year + 2000;
    r->utc_mon   = mon;
    r->utc_day   = day;

    return nmea_reader_update_time( r, time );
}


static double
convert_from_hhmm( Token  tok )
{
    double  val     = str2float(tok.p, tok.end);
    int     degrees = (int)(floor(val) / 100);
    double  minutes = val - degrees*100.;
    double  dcoord  = degrees + minutes / 60.0;
    return dcoord;
}


static int
nmea_reader_update_latlong( NmeaReader*  r,
                            Token        latitude,
                            char         latitudeHemi,
                            Token        longitude,
                            char         longitudeHemi )
{
    double   lat, lon;
    Token    tok;

    tok = latitude;
    if (tok.p + 6 > tok.end) {
        D("latitude is too short: '%.*s'", tok.end-tok.p, tok.p);
        return -1;
    }
    lat = convert_from_hhmm(tok);
    if (latitudeHemi == 'S')
        lat = -lat;

    tok = longitude;
    if (tok.p + 6 > tok.end) {
        D("longitude is too short: '%.*s'", tok.end-tok.p, tok.p);
        return -1;
    }
    lon = convert_from_hhmm(tok);
    if (longitudeHemi == 'W')
        lon = -lon;

    r->fix.flags    |= GPS_LOCATION_HAS_LAT_LONG;
    r->fix.latitude  = lat;
    r->fix.longitude = lon;
    return 0;
}


static int
nmea_reader_update_altitude( NmeaReader*  r,
                             Token        altitude,
                             Token        units )
{
    double  alt;
    Token   tok = altitude;

    if (tok.p >= tok.end)
        return -1;

    r->fix.flags   |= GPS_LOCATION_HAS_ALTITUDE;
    r->fix.altitude = str2float(tok.p, tok.end);
    return 0;
}


static int
nmea_reader_update_bearing( NmeaReader*  r,
                            Token        bearing )
{
    double  alt;
    Token   tok = bearing;

    if (tok.p >= tok.end)
        return -1;

    r->fix.flags   |= GPS_LOCATION_HAS_BEARING;
    r->fix.bearing  = str2float(tok.p, tok.end);
    return 0;
}


static int
nmea_reader_update_speed( NmeaReader*  r,
                          Token        speed )
{
    double  alt;
    Token   tok = speed;

    if (tok.p >= tok.end)
        return -1;

    r->fix.flags   |= GPS_LOCATION_HAS_SPEED;
    r->fix.speed    = str2float(tok.p, tok.end) * 1.825 / 3.6;
    return 0;
}

static int 
nmea_reader_update_accuracy( NmeaReader*  r, 
                             Token        accuracy ) 
{ 
    double  acc; 
    Token   tok = accuracy; 
 
    if (tok.p >= tok.end) 
        return -1; 
 
    r->fix.accuracy = str2float(tok.p, tok.end); 
 
    if (r->fix.accuracy == 99.99){ 
      return 0; 
    } 
 
    r->fix.flags   |= GPS_LOCATION_HAS_ACCURACY; 
    return 0; 
} 

static int
nmea_reader_update_sv_list( GpsSvInfo*  sv,
                            Token        prn,
                            Token        elevation,
                            Token        azimuth,
                            Token        snr)
{
    Token tok = prn;
    if(tok.p >= tok.end){
        memset(sv, 0, sizeof(GpsSvInfo));
        return -1;
    }
    sv->prn = str2int(prn.p, prn.end);
    sv->snr = str2float(snr.p, snr.end);
    sv->elevation = str2float(elevation.p, elevation.end);
    sv->azimuth = str2float(azimuth.p, azimuth.end);
    return 0;
}


static void
nmea_reader_parse( NmeaReader*  r )
{
   /* we received a complete sentence, now parse it to generate
    * a new GPS fix...
    */
    NmeaTokenizer  tzer[1];
    Token          tok;

    D("Received: '%.*s'", r->pos, r->in);
    if (r->pos < 9) {
        D("Too short. discarded.");
        return;
    }

    nmea_tokenizer_init(tzer, r->in, r->in + r->pos);
#if GPS_DEBUG
    {
        int  n;
        D("Found %d tokens", tzer->count);
        for (n = 0; n < tzer->count; n++) {
            Token  tok = nmea_tokenizer_get(tzer,n);
            D("%2d: '%.*s'", n, tok.end-tok.p, tok.p);
        }
    }
#endif

    tok = nmea_tokenizer_get(tzer, 0);
    if (tok.p + 5 > tok.end) {
        D("sentence id '%.*s' too short, ignored.", tok.end-tok.p, tok.p);
        return;
    }

    // ignore first two characters.
    tok.p += 2;
    if ( !memcmp(tok.p, "GGA", 3) ) {
        // GPS fix
        Token  tok_time          = nmea_tokenizer_get(tzer,1);
        Token  tok_latitude      = nmea_tokenizer_get(tzer,2);
        Token  tok_latitudeHemi  = nmea_tokenizer_get(tzer,3);
        Token  tok_longitude     = nmea_tokenizer_get(tzer,4);
        Token  tok_longitudeHemi = nmea_tokenizer_get(tzer,5);
        Token  tok_altitude      = nmea_tokenizer_get(tzer,9);
        Token  tok_altitudeUnits = nmea_tokenizer_get(tzer,10);

        nmea_reader_update_time(r, tok_time);
        nmea_reader_update_latlong(r, tok_latitude,
                                      tok_latitudeHemi.p[0],
                                      tok_longitude,
                                      tok_longitudeHemi.p[0]);
        nmea_reader_update_altitude(r, tok_altitude, tok_altitudeUnits);

    } else if ( !memcmp(tok.p, "GSA", 3) ) {
        int i;
				Token  tok_fixStatus   = nmea_tokenizer_get(tzer, 2); 
				
				if (tok_fixStatus.p[0] != '\0' && tok_fixStatus.p[0] != '1') { 
				  Token  tok_accuracy      = nmea_tokenizer_get(tzer, 15); 
				  nmea_reader_update_accuracy(r, tok_accuracy); 
				} 
				
        for(i = 3; i <= 14; i++){
            Token tok_prn = nmea_tokenizer_get(tzer, i);
            if(tok_prn.p >= tok_prn.end){
                r->status.used_in_fix_mask &= ~(1 << (i-3));
            }else{
                r->status.used_in_fix_mask |= 1 << (i-3);
            }
        }
    } else if ( !memcmp(tok.p, "GSV", 3) ) {
        Token  tok_numgsv          = nmea_tokenizer_get(tzer,1);
        Token  tok_indexgsv          = nmea_tokenizer_get(tzer,2);
        Token  tok_numsvs          = nmea_tokenizer_get(tzer,3);
        int numgsv = str2int(tok_numgsv.p, tok_numgsv.end);
        int indexgsv = str2int(tok_indexgsv.p, tok_indexgsv.end);
        int numsvs = str2int(tok_numsvs.p, tok_numsvs.end);

	if((indexgsv <= numgsv) && (indexgsv >= 1)) //________
        if (numsvs > 0){
            int i, j = 4;
            for(i = (indexgsv-1) << 2; i < indexgsv << 2; i++){
                Token  tok_prn          = nmea_tokenizer_get(tzer,j++);
                Token  tok_elevation          = nmea_tokenizer_get(tzer,j++);
                Token  tok_azimuth          = nmea_tokenizer_get(tzer,j++);
                Token  tok_snr          = nmea_tokenizer_get(tzer,j++);
                nmea_reader_update_sv_list(&r->status.sv_list[i], tok_prn, tok_elevation, tok_azimuth, tok_snr);
            }
            if(indexgsv == numgsv){
                r->status.num_svs = numsvs;
                if(r->sv_status_cb)
                    r->sv_status_cb(&r->status);
            }
        }
    } else if ( !memcmp(tok.p, "RMC", 3) ) {
        Token  tok_time          = nmea_tokenizer_get(tzer,1);
        Token  tok_fixStatus     = nmea_tokenizer_get(tzer,2);
        Token  tok_latitude      = nmea_tokenizer_get(tzer,3);
        Token  tok_latitudeHemi  = nmea_tokenizer_get(tzer,4);
        Token  tok_longitude     = nmea_tokenizer_get(tzer,5);
        Token  tok_longitudeHemi = nmea_tokenizer_get(tzer,6);
        Token  tok_speed         = nmea_tokenizer_get(tzer,7);
        Token  tok_bearing       = nmea_tokenizer_get(tzer,8);
        Token  tok_date          = nmea_tokenizer_get(tzer,9);

        D("in RMC, fixStatus=%c", tok_fixStatus.p[0]);
        if (tok_fixStatus.p[0] == 'A')
        {
            nmea_reader_update_date( r, tok_date, tok_time );

            nmea_reader_update_latlong( r, tok_latitude,
                                           tok_latitudeHemi.p[0],
                                           tok_longitude,
                                           tok_longitudeHemi.p[0] );

            nmea_reader_update_bearing( r, tok_bearing );
            nmea_reader_update_speed  ( r, tok_speed );
        }
    } else {
        tok.p -= 2;
        D("unknown sentence '%.*s", tok.end-tok.p, tok.p);
    }
    if ((r->fix.flags & GPS_LOCATION_HAS_LAT_LONG) &&
           (r->fix.flags & GPS_LOCATION_HAS_ALTITUDE) &&
           (r->fix.flags & (GPS_LOCATION_HAS_SPEED | GPS_LOCATION_HAS_BEARING))) {

#if GPS_DEBUG
        char   temp[256];
        char*  p   = temp;
        char*  end = p + sizeof(temp);
        struct tm   utc;

        p += snprintf( p, end-p, "sending fix" );
        if (r->fix.flags & GPS_LOCATION_HAS_LAT_LONG) {
            p += snprintf(p, end-p, " time=%llu", r->fix.timestamp);
            p += snprintf(p, end-p, " lat=%g lon=%g", r->fix.latitude, r->fix.longitude);
        }
        if (r->fix.flags & GPS_LOCATION_HAS_ALTITUDE) {
            p += snprintf(p, end-p, " altitude=%g", r->fix.altitude);
        }
        if (r->fix.flags & GPS_LOCATION_HAS_SPEED) {
            p += snprintf(p, end-p, " speed=%g", r->fix.speed);
        }
        if (r->fix.flags & GPS_LOCATION_HAS_BEARING) {
            p += snprintf(p, end-p, " bearing=%g", r->fix.bearing);
        }
        if (r->fix.flags & GPS_LOCATION_HAS_ACCURACY) {
            p += snprintf(p,end-p, " accuracy=%g", r->fix.accuracy);
        }
        D("%s",temp);
#endif
        if (r->location_cb) {
            r->location_cb( &r->fix );
            r->fix.flags = 0;
        }
        else {
            D("no location_cb, keeping data until needed !");
        }
    }
}


static void
nmea_reader_addc( NmeaReader*  r, int  c )
{
    if (r->overflow) {
        r->overflow = (c != '\n');
        return;
    }

    if (r->pos >= (int) sizeof(r->in)-1 ) {
        r->overflow = 1;
        r->pos      = 0;
        return;
    }

    r->in[r->pos] = (char)c;
    r->pos       += 1;

    if (c == '\n') {
        nmea_reader_parse( r );
        r->pos = 0;
    }
}


/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       C O N N E C T I O N   S T A T E                 *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/

/* commands sent to the gps thread */
enum {
    CMD_QUIT  = 0,
    CMD_START = 1,
    CMD_STOP  = 2
};


/* this is the state of our connection to the qemu_gpsd daemon */
typedef struct {
    int                     init;
    int                     fd;
#if GPS_GPIO_INCLUDE
    int                     fdGps;          // GPS_GPIO¿ë FD
#endif
    GpsCallbacks            callbacks;
    pthread_t               thread;
    int                     control[2];

} GpsState;

static GpsState  _gps_state[1];


static void
gps_state_done( GpsState*  s )
{
    // tell the thread to quit, and wait for it
    char   cmd = CMD_QUIT;
    void*  dummy;
    char	buf[10];
    int ret;
    
    write( s->control[0], &cmd, 1 );
    pthread_join(s->thread, &dummy);

    // close the control socket pair
    close( s->control[0] ); s->control[0] = -1;
    close( s->control[1] ); s->control[1] = -1;

    // close connection to the QEMU GPS daemon
    close( s->fd ); s->fd = -1;
    s->init = 0;
    
#if GPS_GPIO_INCLUDE
		if(s->fdGps>0)
		{
    	ret = sprintf(buf,"%d\n",0);
 			ret = write(s->fdGps, buf, ret);
    	close( s->fdGps ); s->fdGps = -1;
    }
#endif
}


static void
gps_state_start( GpsState*  s )
{
    char  cmd = CMD_START;
    int   ret;
    char	buf[10];

    do { ret=write( s->control[0], &cmd, 1 ); }
    while (ret < 0 && errno == EINTR);

    if (ret != 1)
        D("%s: could not send CMD_START command: ret=%d: %s",
          __FUNCTION__, ret, strerror(errno));
          
#if GPS_GPIO_INCLUDE
		if(s->fdGps>0)
		{
    	ret = sprintf(buf,"%d\n",1);
 			ret = write(s->fdGps, buf, ret);
 		}
#endif
}


static void
gps_state_stop( GpsState*  s )
{
    char  cmd = CMD_STOP;
    int   ret;
    char	buf[10];

    do { ret=write( s->control[0], &cmd, 1 ); }
    while (ret < 0 && errno == EINTR);

    if (ret != 1)
        D("%s: could not send CMD_STOP command: ret=%d: %s",
          __FUNCTION__, ret, strerror(errno));
#if GPS_GPIO_INCLUDE
		if(s->fdGps>0)
		{
    	ret = sprintf(buf,"%d\n",0);
 			ret = write(s->fdGps, buf, ret);
 		}
#endif
}


static int
epoll_register( int  epoll_fd, int  fd )
{
    struct epoll_event  ev;
    int           ret, flags;

    /* important: make the fd non-blocking */
    flags = fcntl(fd, F_GETFL);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    ev.events  = EPOLLIN;
    ev.data.fd = fd;
    do {
        ret = epoll_ctl( epoll_fd, EPOLL_CTL_ADD, fd, &ev );
    } while (ret < 0 && errno == EINTR);
    return ret;
}


static int
epoll_deregister( int  epoll_fd, int  fd )
{
    int  ret;
    do {
        ret = epoll_ctl( epoll_fd, EPOLL_CTL_DEL, fd, NULL );
    } while (ret < 0 && errno == EINTR);
    return ret;
}

/* this is the main thread, it waits for commands from gps_state_start/stop and,
 * when started, messages from the QEMU GPS daemon. these are simple NMEA sentences
 * that must be parsed to be converted into GPS fixes sent to the framework
 */
static void*
gps_state_thread( void*  arg )
{
    GpsState*   state = (GpsState*) arg;
    NmeaReader  reader[1];
    int         epoll_fd   = epoll_create(2);
    int         started    = 0;
    int         gps_fd     = state->fd;
    int         control_fd = state->control[1];

    nmea_reader_init( reader );

    // register control file descriptors for polling
    epoll_register( epoll_fd, control_fd );
    epoll_register( epoll_fd, gps_fd );

    D("gps thread running");

    // now loop
    for (;;) {
        struct epoll_event   events[2];
        int                  ne, nevents;

        nevents = epoll_wait( epoll_fd, events, 2, -1 );
        if (nevents < 0) {
            if (errno != EINTR)
                LOGE("epoll_wait() unexpected error: %s", strerror(errno));
            continue;
        }
        D("gps thread received %d events", nevents);
        for (ne = 0; ne < nevents; ne++) {
            if ((events[ne].events & (EPOLLERR|EPOLLHUP)) != 0) {
                LOGE("EPOLLERR or EPOLLHUP after epoll_wait() !?");
                goto Exit;
            }
            if ((events[ne].events & EPOLLIN) != 0) {
                int  fd = events[ne].data.fd;

                if (fd == control_fd)
                {
                    char  cmd = 255;
                    int   ret;
                    D("gps control fd event");
                    do {
                        ret = read( fd, &cmd, 1 );
                    } while (ret < 0 && errno == EINTR);

                    if (cmd == CMD_QUIT) {
                        D("gps thread quitting on demand");
                        goto Exit;
                    }
                    else if (cmd == CMD_START) {
                        if (!started) {
                            D("gps thread starting  location_cb=%p", state->callbacks.location_cb);
                            started = 1;
                            nmea_reader_set_callback( reader, state->callbacks.location_cb, state->callbacks.sv_status_cb );
                        }
                    }
                    else if (cmd == CMD_STOP) {
                        if (started) {
                            D("gps thread stopping");
                            started = 0;
                            nmea_reader_set_callback( reader, NULL, NULL );
                        }
                    }
                }
                else if (fd == gps_fd)
                {
                    char  buff[32];
                    D("gps fd event");
                    for (;;) {
                        int  nn, ret;

                        ret = read( fd, buff, sizeof(buff) );
                        if (ret < 0) {
                            if (errno == EINTR)
                                continue;
                            if (errno != EWOULDBLOCK)
                                LOGE("error while reading from gps daemon socket: %s:", strerror(errno));
                            break;
                        }
                        D("received %d bytes: %.*s", ret, ret, buff);
                        for (nn = 0; nn < ret; nn++)
                            nmea_reader_addc( reader, buff[nn] );
                    }
                    D("gps fd event end");
                }
                else
                {
                    LOGE("epoll_wait() returned unkown fd %d ?", fd);
                }
            }
        }
    }
Exit:
    return NULL;
}


static void
gps_state_init( GpsState*  state )
{
		char   prop[PROPERTY_VALUE_MAX]; 
		char   device[256];
		char 	 buf[10];
    int 	 ret;
		
    state->init       = 0;
    state->control[0] = -1;
    state->control[1] = -1;
    state->fd         = -1;
#if GPS_GPIO_INCLUDE
    state->fdGps      = -1;
#endif

		LOGI("gps_state_init"); 
		
		if (property_get("ro.machine.type",prop,"") == 0) {
        LOGE("no ro.machine.type prop");
    }
    
//    LOGD("ro.machine.type=%s",prop);
    
//     if(memcmp(prop,"yho",3))
//    {
//    	 LOGE("no ro.machine.type is not yho");
//    	 return;
//    }
    
    sprintf(device, "/dev/%s", GPS_CHANNEL_NAME);

	state->fd = open(device, O_RDONLY);
	if(state->fd < 0){
		LOGE("No gps hardware detected (Device %s not found!)", device);
		return;
	}
	{
	    struct termios  ios;
	    tcgetattr( state->fd, &ios );
	    ios.c_lflag = 0;  /* disable ECHO, ICANON, etc... */
	    cfsetospeed(&ios, B9600);
	    cfsetispeed(&ios, B9600);
	    tcsetattr( state->fd, TCSANOW, &ios );
	}
	
#if GPS_GPIO_INCLUDE
    state->fdGps = open(GPS_GPIO_DEV_NAME, O_RDWR);                               // Gps_GPIO
    LOGD("starview : Gps_GPIO Device Open FDescriptor %d",state->fdGps);

    if (state->fdGps < 0) {
        LOGE("starview : Couldn't open gps_gpio");
        close( state->fd ); 
        state->fd = -1;  
        return;
    }
    ret = sprintf(buf,"%d\n",1);
    ret = write(state->fdGps, buf, ret);
#endif

    if ( socketpair( AF_LOCAL, SOCK_STREAM, 0, state->control ) < 0 ) {
        LOGE("could not create thread control socket pair: %s", strerror(errno));
        goto Fail;
    }
#if 0
    if ( pthread_create( &state->thread, NULL, gps_state_thread, state ) != 0 ) {
        LOGE("could not create gps thread: %s", strerror(errno));
        goto Fail;
    }
#else
		state->callbacks.create_thread_cb("qemu_gps",  gps_state_thread, state);
#endif
		state->init = 1;
    D("gps state initialized");
    return;

Fail:
    gps_state_done( state );
}


/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       I N T E R F A C E                               *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/


static int
gps_init(GpsCallbacks* callbacks)
{
    GpsState*  s = _gps_state;
    s->callbacks = *callbacks;

    if (!s->init)
        gps_state_init(s);

    if (s->fd < 0)
        return -1;

    //s->callbacks = *callbacks;

    return 0;
}

static void
gps_cleanup(void)
{
    GpsState*  s = _gps_state;

    if (s->init)
        gps_state_done(s);
}


static int
gps_start()
{
    GpsState*  s = _gps_state;

    if (!s->init) {
        D("%s: called with uninitialized state !!", __FUNCTION__);
        return -1;
    }

    D("%s: called", __FUNCTION__);
    gps_state_start(s);
    return 0;
}


static int
gps_stop()
{
    GpsState*  s = _gps_state;

    if (!s->init) {
        D("%s: called with uninitialized state !!", __FUNCTION__);
        return -1;
    }

    D("%s: called", __FUNCTION__);
    gps_state_stop(s);
    return 0;
}


static int
gps_inject_time(GpsUtcTime time, int64_t timeReference, int uncertainty)
{
    return 0;
}

static void
gps_delete_aiding_data(GpsAidingData flags)
{
}

static int gps_set_position_mode(GpsPositionMode mode, int fix_frequency)
{
    // FIXME - support fix_frequency
    // only standalone supported for now.
    if (mode != GPS_POSITION_MODE_STANDALONE)
        return -1;
    return 0;
}

static const void*
gps_get_extension(const char* name)
{
    return NULL;
}

static const GpsInterface  odroidGpsInterface = {
		sizeof(GpsInterface),
    .init = gps_init,
    .start = gps_start,
    .stop = gps_stop,
    .cleanup = gps_cleanup,
    .inject_time = gps_inject_time,
    .delete_aiding_data = gps_delete_aiding_data,
    .set_position_mode = gps_set_position_mode,
    .get_extension = gps_get_extension,
};

//const GpsInterface* gps_get_interface()
//{
 //   return &_GpsInterface;
//}
const GpsInterface* gps__get_gps_interface(struct gps_device_t* dev)
{
    return &odroidGpsInterface;
}

static int open_gps(const struct hw_module_t* module, char const* name,
        struct hw_device_t** device)
{
    struct gps_device_t *dev = malloc(sizeof(struct gps_device_t));
    memset(dev, 0, sizeof(*dev));

    dev->common.tag = HARDWARE_DEVICE_TAG;
    dev->common.version = 0;
    dev->common.module = (struct hw_module_t*)module;
//    dev->common.close = (int (*)(struct hw_device_t*))close_lights;
    dev->get_gps_interface = gps__get_gps_interface;

    *device = (struct hw_device_t*)dev;
    return 0;
}


static struct hw_module_methods_t gps_module_methods = {
    .open = open_gps
};

const struct hw_module_t HAL_MODULE_INFO_SYM = {
    .tag = HARDWARE_MODULE_TAG,
    .version_major = 1,
    .version_minor = 0,
    .id = GPS_HARDWARE_MODULE_ID,
    .name = "Goldfish GPS Module",
    .author = "The Android Open Source Project",
    .methods = &gps_module_methods,
};

#define SECONDS_PER_MIN (60)
#define SECONDS_PER_HOUR (60*SECONDS_PER_MIN)
#define SECONDS_PER_DAY  (24*SECONDS_PER_HOUR)
#define SECONDS_PER_NORMAL_YEAR (365*SECONDS_PER_DAY) 
#define SECONDS_PER_LEAP_YEAR (SECONDS_PER_NORMAL_YEAR+SECONDS_PER_DAY) 


static int days_per_month_no_leap[] =
    {31,28,31,30,31,30,31,31,30,31,30,31};
static int days_per_month_leap[] =
    {31,29,31,30,31,30,31,31,30,31,30,31};

static int is_leap_year(int year)
{
    if ((year%400) == 0)
        return 1;
    if ((year%100) == 0)
        return 0;
    if ((year%4) == 0)
        return 1;
    return 0;
}
static int number_of_leap_years_in_between(int from, int to)
{
    int n_400y, n_100y, n_4y;
    n_400y = to/400 - from/400;
    n_100y = to/100 - from/100;
    n_4y = to/4 - from/4;
    
    // if this year is leap, we must remove it. 
    if ((to%4)==0)
    	--n_4y;
    	
    return (n_4y - n_100y + n_400y);
}

static time_t utc_mktime(struct tm *_tm)
{
    time_t t_epoch=0;
    int m; 
    int *days_per_month;
    if (is_leap_year(_tm->tm_year+1900))
        days_per_month = days_per_month_leap;
    else
        days_per_month = days_per_month_no_leap;
    t_epoch += (_tm->tm_year - 70)*SECONDS_PER_NORMAL_YEAR; 
    t_epoch += number_of_leap_years_in_between(1970,_tm->tm_year+1900) *
        SECONDS_PER_DAY;
    for (m=0; m<_tm->tm_mon; m++) {
        t_epoch += days_per_month[m]*SECONDS_PER_DAY;
    }
    t_epoch += (_tm->tm_mday-1)*SECONDS_PER_DAY;
    t_epoch += _tm->tm_hour*SECONDS_PER_HOUR;
    t_epoch += _tm->tm_min*SECONDS_PER_MIN;
    t_epoch += _tm->tm_sec;
    return t_epoch;

}
