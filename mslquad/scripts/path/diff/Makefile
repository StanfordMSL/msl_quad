all: test

test: test.o diff2SttInC.o dMath.o diffMath.o
	gcc test.o diff2SttInC.o dMath.o diffMath.o -o test

test.o: test.c 
	gcc -c test.c -I.
#if need to debug
#	gcc -c -g test.c -I.

diff2SttInC.o: diff2SttInC.c 
	gcc -c diff2SttInC.c -I.
#if need to debug
#	gcc -c -g diff2SttInC.c -I.

dMath.o: dMath.c
	gcc -c dMath.c -I.

diffMath.o: diffMath.c
	gcc -c diffMath.c -I.
	
clean:
	rm -f *.o test
