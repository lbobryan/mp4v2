CC=g++
CFLAGS += -Werror -Wextra -Wall -g
LDLIBS := $(shell pkg-config --libs mp4v2) $(LDLIBS)

# Debug target to print variables.
print-%:
	@echo '$*=$($*)'

all: camm_muxing.o
	$(CC) camm_muxing.o $(LDLIBS) $(CFLAGS) -o camm_muxing

clean:
	rm camm_muxing camm_muxing.o

# You may need to add `/usr/local/lib` to the `LD_LIBRARY_PATH` environment
# variable to ensure shared library is accessible during execution.
#
# You may need to add the package config for mp4v2 to your system's package
# config location.
# For example, you can add `/usr/local/lib/pkgconfig` to the `PKG_CONFIG_PATH`
# environment variable and put the following package config (remove all '#'s) in
# "/usr/local/lib/pkgconfig/mp4v2.pc".
#
#prefix=/usr/local
#exec_prefix=${prefix}
#libdir=/usr/local/lib
#includedir=/usr/local/include
#
#Name: mp4v2
#Description: MP4v2 library
#Version: 1.0.0
#Requires:
#Requires.private:
#Conflicts:
#Libs: -L${libdir} -lmp4v2
#Libs.private:
#Cflags: -I${includedir}
#
# In the end, try "pkg-config --libs mp4v2" to verify the setup.
