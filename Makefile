.PHONY: deps image build test sim experiments plots check

DEPS  ?= tsb-control-deps
IMAGE ?= tsb-control
MISSION ?= square

# BuildKit when the buildx plugin is installed, the classic builder otherwise, since not
# every Docker install has buildx. BUILD_FLAGS passes extra options: CI uses the host's
# network, because inside the build's own network GitHub's runners could not resolve
# archive.ubuntu.com.
DOCKER_BUILDKIT ?= $(shell docker buildx version >/dev/null 2>&1 && echo 1 || echo 0)
BUILD_FLAGS ?=
BUILD = DOCKER_BUILDKIT=$(DOCKER_BUILDKIT) docker build $(BUILD_FLAGS) -f docker/Dockerfile
RUN   = docker run --rm -e HOME=/tmp -v /etc/passwd:/etc/passwd:ro -u $$(id -u):$$(id -g) \
        -v "$(CURDIR):/ws" -w /ws $(DEPS) bash -c
ROS   = source /opt/ros/humble/setup.bash
WS    = $(ROS) && source install/setup.bash
COLCON_ARGS = --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

deps:         ## Build the image with ROS 2 Humble, Gazebo and TurtleBot3 (once, about 5 min).
	$(BUILD) --target deps -t $(DEPS) .

image:        ## Build a self-contained image that also compiles and tests the workspace.
	$(BUILD) -t $(IMAGE) .

build:        ## Compile the workspace into build/ and install/ using the deps image.
	$(RUN) '$(ROS) && colcon build $(COLCON_ARGS)'

test: build   ## Unit, lint and launch tests.
	$(RUN) '$(WS) && colcon test --event-handlers console_direct+; colcon test-result --verbose'

sim: build    ## Headless Gazebo running a mission: make sim MISSION=zigzag
	docker run --rm -it -e HOME=/tmp -v /etc/passwd:/etc/passwd:ro -u $$(id -u):$$(id -g) \
	  -v "$(CURDIR):/ws" -w /ws $(DEPS) bash -c \
	  '$(WS) && ros2 launch tsb_control sim.launch.py mission:=$(MISSION)'

experiments: build  ## Record every run in Gazebo and regenerate docs/results (about 2 min).
	experiments/run.sh

plots:        ## Regenerate docs/results from the bags already in experiments/bags.
	$(RUN) '$(WS) && python3 experiments/replot.py'

check:        ## Compare docs/results against the regression thresholds.
	python3 experiments/check.py
