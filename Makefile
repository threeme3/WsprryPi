all:
	gcc -D _BSD_SOURCE -lm -std=c99 wspr.c -owspr

clean:
	rm wspr
