#!/system/bin/busybox sh

log (){
echo $1 >>/data/ZX803_init.log
}

log1 (){
echo $1 
}

cp_wait (){
	log "busybox cp "$1" to "$2
	while [ ! -f $2 ]
	do
		log "waiting for install: "$2
		sleep 2
	done 
	
	busybox cp -rf $1 $2
}
config (){
	log "busybox cp "$1" to "$2
	busybox cp $1 $2
}

echo "1">>/data/start

if [ -f /data/start_flag ]; then
	echo "start_flag"
	
	exit 0
fi

mount -o,remount,rw /system
chmod -R 777 /system/config/*

mkdir /data/InterNav
chmod 777 /data/InterNav

busybox cp -p /system/config/cfg.xml /data/InterNav/cfg.xml
log "/system/config/cfg.xml /data/InterNav/cfg.xml" 
cp -f /system/lib/libmeida1.so /system/lib/libmeida.so 
cp -f /system/lib/libinputservice1.so /system/lib/libinputservice.so

sleep 2

log "config completed!"
echo "">/data/start_flag


