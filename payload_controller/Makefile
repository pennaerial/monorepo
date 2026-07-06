.PHONY: sitl esp32s3

sitl:
	@if [ ! -f build/linux/CMakeCache.txt ]; then \
		echo "Initializing Linux build directory..."; \
		idf.py -B build/linux -D SDKCONFIG=sdkconfig.linux --preview set-target linux; \
	fi
	idf.py -B build/linux build

esp32s3:
	@if [ ! -f build/esp32s3/CMakeCache.txt ]; then \
		echo "Initializing ESP32-S3 build directory..."; \
		idf.py -B build/esp32s3 -D SDKCONFIG=sdkconfig.esp32s3 set-target esp32s3; \
	fi
	idf.py -B build/esp32s3 build
