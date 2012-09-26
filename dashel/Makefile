all: dashel

dashel:
	mkdir -p build
	cd build && cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX=. ../upstream_src
	cd build && make
	cd build && make install
	touch dashel

clean:
	-rm -rf build dashel include lib share src