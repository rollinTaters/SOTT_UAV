RELEASE_PATH := release_builds

.PHONY: release

# ==== Release commands ====
release:
	rm -r -f $(RELEASE_PATH)
	mkdir $(RELEASE_PATH)
	cp -r ./raylib-5.5_linux_amd64/lib $(RELEASE_PATH)/
	make -C common_code release
	make -C flight_sim release
	make -C command_console release
	make -C mc release
	
run:
	cd $(RELEASE_PATH) && ./CC_linux&
	cd $(RELEASE_PATH) && ./FS_linux&
