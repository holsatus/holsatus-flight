set shell := ["bash", "-euo", "pipefail", "-c"]

_cargo_release COMMAND:
	echo "> cargo build --release {{COMMAND}}"
	cargo build --release {{COMMAND}}

build-features:
	for features in "" "sdmmc" "usb" "mavlink" "sdmmc usb" "sdmmc mavlink" "usb mavlink" "sdmmc usb mavlink"; do \
		if [ -z "$$features" ]; then \
			echo "\n==> Building with no optional features"; \
			just _cargo_release ""; \
		else \
			echo "\n==> Building with features: $$features"; \
			just _cargo_release "--features \"$$features\""; \
		fi; \
	done
