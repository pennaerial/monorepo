.PHONY: linux esp32s3 set_cc

linux esp32s3:
	@if [ ! -f build/$@/CMakeCache.txt ]; then \
		echo "Initializing $@ build directory..."; \
		idf.py -B build/$@ -D SDKCONFIG=sdkconfig.$@ $(if $(filter linux,$@),--preview) set-target $@; \
	fi
	idf.py -B build/$@ -D SDKCONFIG=sdkconfig.$@ build
	$(MAKE) --no-print-directory set_cc TARGET=$@

set_cc:
	@case "$(TARGET)" in linux|esp32s3) ;; *) \
		echo "Usage: make set_cc TARGET=linux|esp32s3" >&2; exit 1 ;; esac
	@test -f "build/$(TARGET)/compile_commands.json" || { \
		echo "Run 'make $(TARGET)' first to generate compile commands." >&2; exit 1; }
	ln -sf build/$(TARGET)/compile_commands.json compile_commands.json

WORLD ?= default

# `make linux gz_<model>` builds, then spawns gz-models/models/<model> into
# gz-models/worlds/$(WORLD).sdf and runs SITL against it (HEADLESS=1 for no GUI)
gz_%: linux
	./scripts/sim.sh $* $(WORLD)

