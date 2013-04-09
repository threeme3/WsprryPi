all:
	gcc -lm -std=c99 wspr.c -owspr

clean:
	rm wspr
