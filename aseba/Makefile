all: aseba

aseba:
	mkdir -p build
	cd build && cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX=.. -DDASHEL_INCLUDE_DIR=`rospack cflags-only-I dashel` -DCMAKE_LIBRARY_PATH=`rospack libs-only-L dashel` ../upstream_src
	cd build && make
	cd build && make install
	touch aseba

clean:
	-rm -rf build aseba include lib share src