IMACSROOT = /home/imacs/Desktop/imacs
HALIDE_PATH = /home/imacs/Desktop/imacs/externalApps/Halide
OPENCV_PATH = /home/imacs/Desktop/imacs/externalApps/opencv
PKG_CONFIG_PATH =/usr/lib/pkgconfig

all:


gen:
	g++  src/nl_means_generator.cpp $(HALIDE_PATH)/tools/GenGen.cpp -g \
	-I $(HALIDE_PATH)/include \
	-I $(HALIDE_PATH)/tools \
	-L $(HALIDE_PATH)/bin -lHalide \
	-L /usr/local/lib -ljpeg \
	`libpng-config --cflags --ldflags` \
	-fno-rtti -lpthread -ldl -lm \
	-o nl_means_generator.o -std=c++11
	
run_denoise_multiple_chrono:
	g++ src/denoise_multiple.cpp \
	-I $(HALIDE_PATH)/include \
	-I $(HALIDE_PATH)/tools \
	-I $(OPENCV_PATH)/include \
	-L $(HALIDE_PATH)/bin -lHalide \
	-L $(OPENCV_PATH)/lib \
	nl_means.a \
	-L /usr/local/lib -ljpeg \
	`libpng-config --cflags --ldflags`  \
	`pkg-config opencv --cflags --libs` -lpthread -ldl -lm -DPROFILEWITHCHRONO \
	-o denoise_multiple.o -std=c++11

run_denoise_single_chrono:
	g++ src/denoise_single.cpp \
	-I $(HALIDE_PATH)/include \
	-I $(HALIDE_PATH)/tools \
	-I $(OPENCV_PATH)/include \
	-L $(HALIDE_PATH)/bin -lHalide \
	-L $(OPENCV_PATH)/lib \
	nl_means.a \
	-L /usr/local/lib -ljpeg \
	`libpng-config --cflags --ldflags`  \
	`pkg-config opencv --cflags --libs` -lpthread -ldl -lm -DPROFILEWITHCHRONO \
	-o denoise_single.o -std=c++11
	
run_denoise_single_valgrind:
	g++ src/denoise_single.cpp \
	-I /usr/include/valgrind \
	-I $(HALIDE_PATH)/include \
	-I $(HALIDE_PATH)/tools \
	-I $(OPENCV_PATH)/include \
	-L $(HALIDE_PATH)/bin -lHalide \
	-L $(OPENCV_PATH)/lib \
	nl_means.a \
	-L /usr/local/lib -ljpeg \
	`libpng-config --cflags --ldflags`  \
	`pkg-config opencv --cflags --libs` -lpthread -ldl -lm -DPROFILEWITHVALGRIND \
	-o denoise_single.o -std=c++11

clean:
	rm -f *.o nl_means.*
