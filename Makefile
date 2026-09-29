# PlatformIO Wrapper Makefile
# Thin wrapper over PlatformIO (pio) for common development tasks.
#
# NOTE: Arduino Uno (uno), Uno R4 WiFi (uno_r4_wifi) and ATtiny85 (attiny85) support only the 'blink' program.
# ATtiny85 has no hardware serial, so monitor is unavailable; uses ISP programmer.
# NOTE: tilt_servo supports only XIAO ESP32-C3 (xiao_c3); it is not built for other boards.

# pio from PATH, else the user-level PlatformIO install, else plain `pio`.
# Override with e.g. `make PIO=.venv/bin/pio build`.
PIO ?= $(or $(shell command -v pio 2>/dev/null),$(wildcard $(HOME)/.platformio/penv/bin/pio),pio)
ENV ?= balance-xiao_s3
PORT ?=

UPLOAD_PORT := $(if $(PORT),--upload-port $(PORT))
MONITOR_PORT := $(if $(PORT),--port $(PORT))

ENVS := balance-xiao_s3 balance-esp32dev blink-uno blink-uno_r4_wifi blink-attiny85 blink-xiao_s3 blink-esp32dev tilt_servo-xiao_c3

.DEFAULT_GOAL := help
.PHONY: help build upload monitor clean test build-all clean-all envs graph ports port check build-% upload-% monitor-% clean-%

# Build the default (or specified) environment.
build: ## Build ENV (default: balance-xiao_s3)
	$(PIO) run -e $(ENV)

# Upload firmware to device on default (or specified) environment.
upload: ## Upload ENV (optional: PORT=/dev/ttyUSBX)
	$(PIO) run -e $(ENV) -t upload $(UPLOAD_PORT)

# Monitor serial output from default (or specified) environment.
monitor: ## Monitor serial output for ENV (optional: PORT=/dev/ttyUSBX)
	$(PIO) device monitor -e $(ENV) $(MONITOR_PORT)

# Clean build artifacts for default (or specified) environment.
clean: ## Clean build artifacts for ENV
	$(PIO) run -e $(ENV) -t clean

# Run native unit tests.
test: ## Run unit tests (native platform)
	$(PIO) test -e test

# Build all environments (stops on first failure).
build-all: ## Build all ENVS sequentially
	@for env in $(ENVS); do \
		echo "Building $$env..."; \
		$(PIO) run -e $$env || exit 1; \
	done

# Clean all build artifacts.
clean-all: ## Clean all build artifacts
	$(PIO) run -t clean

# List all supported environments.
envs: ## List all supported environments
	@echo "Supported environments:"; \
	for env in $(ENVS); do echo "  $$env"; done

# Rebuild code-review-graph and graphify knowledge graphs.
graph: ## Rebuild knowledge graphs (code-review-graph, graphify)
	code-review-graph build && graphify update .

# USB serial ports only: pio prints "Hardware ID: USB VID:PID=..." for real USB devices, "n/a" for /dev/ttyS* noise.
DETECT_PORTS = $(PIO) device list 2>/dev/null | awk '/^\/dev\// {p=$$0} /Hardware ID:.*USB VID:PID=/ {print p}'

# List detected USB serial ports.
ports: ## List detected USB serial ports (ignores /dev/ttyS*)
	@ports=$$($(DETECT_PORTS)); \
	if [ -z "$$ports" ]; then echo "No USB serial ports detected. Connect the board (XIAO: hold BOOT while plugging in)." >&2; exit 1; fi; \
	echo "Detected USB serial ports:"; printf '  %s\n' $$ports

# Print the single port to use for upload: PORT if set, else the one detected port.
port: ## Print serial port to use for upload (PORT, else auto-detected)
	@if [ -n "$(PORT)" ]; then echo "$(PORT)"; exit 0; fi; \
	ports=$$($(DETECT_PORTS)); \
	n=$$(printf '%s\n' "$$ports" | grep -c .); \
	if [ "$$n" -eq 0 ]; then echo "No USB serial port found. Connect the board or pass PORT=/dev/ttyXXX." >&2; exit 1; fi; \
	if [ "$$n" -gt 1 ]; then echo "Multiple ports found; choose one with PORT=...:" >&2; printf '  %s\n' $$ports >&2; exit 1; fi; \
	echo "$$ports"

# Check the dev board is reachable: port resolved, exists, readable and writable.
check: ## Check board connectivity for ENV (optional: PORT=/dev/ttyUSBX)
	@case "$(ENV)" in *attiny85*) echo "$(ENV): ISP upload, no serial port; connectivity check not supported."; exit 0;; esac; \
	p=$$($(MAKE) --no-print-directory -s port) || exit 1; \
	[ -e "$$p" ] || { echo "FAIL: $$p does not exist" >&2; exit 1; }; \
	{ [ -r "$$p" ] && [ -w "$$p" ]; } || { echo "FAIL: no read/write access to $$p. Fix: sudo usermod -aG dialout $$USER, then re-login." >&2; exit 1; }; \
	echo "OK: $$p present and accessible (ENV=$(ENV))"

# Pattern rules: build-<env>, upload-<env>, monitor-<env>, clean-<env>
# Example: make build-blink-uno, make upload-balance-xiao_s3 PORT=/dev/ttyUSB0
build-%: ## Build specific env (e.g., build-blink-uno)
	$(PIO) run -e $*

upload-%: ## Upload specific env (e.g., upload-blink-uno)
	$(PIO) run -e $* -t upload $(UPLOAD_PORT)

monitor-%: ## Monitor specific env (e.g., monitor-blink-uno)
	$(PIO) device monitor -e $* $(MONITOR_PORT)

clean-%: ## Clean specific env (e.g., clean-blink-uno)
	$(PIO) run -e $* -t clean

# Help: display available targets and usage.
help: ## Display this help message
	@echo "PlatformIO Makefile — Development targets"
	@echo ""
	@echo "Usage Examples:"
	@echo "  make build                              # Build default env: $(ENV)"
	@echo "  make build ENV=blink-uno                # Build specific env"
	@echo "  make upload ENV=blink-uno               # Upload to device"
	@echo "  make upload ENV=blink-uno PORT=/dev/ttyUSB0 # Upload to specific port"
	@echo "  make upload-blink-uno                   # Upload via pattern rule"
	@echo "  make monitor-blink-uno                  # Monitor via pattern rule"
	@echo "  make ports                              # List detected USB serial ports"
	@echo "  make port                               # Print port to use for upload"
	@echo "  make check ENV=tilt_servo-xiao_c3       # Check board connectivity"
	@echo "  make upload PORT=\$$(make -s port)       # Upload using detected port"
	@echo ""
	@echo "Targets:"
	@grep -E '^[a-z_%-]+:.*## ' $(MAKEFILE_LIST) | awk -F ':.*## ' '{printf "  %-12s %s\n", $$1, $$2}'
	@echo ""
	@echo "Supported Environments (ENVS):"
	@for env in $(ENVS); do echo "  $$env"; done
	@echo ""
	@echo "Configuration:"
	@echo "  PIO   = $(PIO)"
	@echo "  ENV   = $(ENV)  [default environment]"
	@echo "  PORT  = $(if $(PORT),$(PORT),[unset - optional for upload/monitor])"
	@echo ""
	@echo "Notes:"
	@echo "  - Arduino Uno (uno), Uno R4 WiFi (uno_r4_wifi) and ATtiny85 (attiny85) support only 'blink' program."
	@echo "  - ATtiny85 has no hardware serial; monitor is unavailable."
	@echo "  - ATtiny85 requires ISP programmer for upload."
	@echo "  - tilt_servo supports only XIAO ESP32-C3 (tilt_servo-xiao_c3)."
	@echo "  - 'make ports'/'port'/'check' find USB serial boards; with several attached, pass PORT=..."
	@echo "  - Use PORT=/dev/ttyUSBX to specify a custom serial port."
