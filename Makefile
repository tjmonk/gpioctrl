INCLUDE_DIRS=-I../libvarserver/inc \
			 -I../libjson/inc \
			 -I./inc \
			 $(shell pkg-config --cflags libgpiod)

LIB_DIRS=../libvarserver/ ../libjson/
GPIOD_LIBS=$(shell pkg-config --libs libgpiod)

all: docs gpioctrl

clean:
	rm -f *.o
	rm -f *.so
	rm -f getvar
	rm -rf doc/latex
	rm -rf doc/html

docs: gpioctrl
	doxygen doc/Doxyfile
	cd doc/latex && make

gpioctrl: gpioctrl.o
	gcc gpioctrl.o -L ${LIB_DIRS} -lvarserver -lrt -lpthread -ltjson ${GPIOD_LIBS} -o gpioctrl

gpioctrl.o: src/gpioctrl.c
	gcc -c ${INCLUDE_DIRS} src/gpioctrl.c -o gpioctrl.o
