INCLUDE_DIRS=-I../libvarserver/inc \
			 -I../libjson/inc \
			 -I./inc

LIB_DIRS=../libvarserver/ ../libjson/

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
	gcc gpioctrl.o -L ${LIB_DIRS} -lvarserver -lrt -lpthread -ltjson -lgpiod -o gpioctrl

gpioctrl.o: src/gpioctrl.c
	gcc -c ${INCLUDE_DIRS} src/gpioctrl.c -o gpioctrl.o
