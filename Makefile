all:
	gcc -D _BSD_SOURCE -lm -std=c99 wspr.c -owspr

clean:
	rm wspr

install:
	cp wspr /usr/local/sbin/

uninstall:
	rm /usr/local/sbin/wspr
