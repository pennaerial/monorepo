.PHONY: linux esp32s3 set_cc

linux esp32s3:
	@if [ ! -f build/$@/CMakeCache.txt ]; then \
		echo "Initializing $@ build directory..."; \
		idf.py -B build/$@ -D SDKCONFIG=sdkconfig.$@ $(if $(filter linux,$@),--preview) set-target $@; \
	fi
	idf.py -B build/$@ -D SDKCONFIG=sdkconfig.$@ build

set_cc:
	ln -sf build/$(TARGET)/compile_commands.json compile_commands.json
