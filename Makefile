
all: test test2

bidshot.pio.h: bidshot.pio
	pioasm bidshot.pio bidshot.pio.h

test: bidshot.pio.h test.c motor-bidshot.c piolib.c pio_rp1.c
	gcc -O2 -Iinclude -Wall -o test test.c motor-bidshot.c piolib.c pio_rp1.c

test2: bidshot.pio.h test2.c motor-bidshot.c piolib.c pio_rp1.c
	gcc -O2 -Iinclude -Wall -o test2 test2.c motor-bidshot.c piolib.c pio_rp1.c

clean: always
	rm -f *~ test test2

run: always
	# If you need to reset ESC, uncomment the following
	# pinctrl set 19 op dl
	# sleep 5
	sudo ./test 19


.PHONY: always



