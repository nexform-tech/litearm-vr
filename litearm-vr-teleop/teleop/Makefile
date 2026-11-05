# litearm-vr-teleop 项目常用命令
# 用法: make <target>
#
# metavr-arm 环境的解释器按这个顺序定，与 launch/run_teleop_webxr.py 同一套逻辑：
#   1) 显式指定：make PY_ARM=/path/to/python <target>   或   传 --metavr-py
#   2) 自动探测常见 conda/venv 根（setup.sh 建的就在其中之一）
CONDA_ROOTS := $(HOME)/sl/miniconda3 $(HOME)/miniconda3 $(HOME)/opt/anaconda3 $(HOME)/anaconda3
# 两种布局都探：conda 是 <root>/envs/metavr-arm/bin/python，venv 是 ~/venvs/metavr-arm/bin/python
METAVR_PY_ROOTS := $(addsuffix /envs/metavr-arm/bin/python,$(CONDA_ROOTS)) \
                   $(HOME)/venvs/metavr-arm/bin/python
METAVR_PY_AUTO := $(firstword $(wildcard $(METAVR_PY_ROOTS)))
PY_ARM ?= $(if $(METAVR_PY),$(METAVR_PY),$(METAVR_PY_AUTO))

.PHONY: help teleop stop smoke test test-exact test-config test-vr test-wrist-slew setup-env check-py

help:
	@echo "make setup-env  重建 conda 环境（metavr-arm）"
	@echo "make teleop     一键启动 LiteArm VR 遥操服务端（参数直通，见 ARGS）"
	@echo "make stop       停止遥操服务端"
	@echo "make smoke      LiteArm pybullet 环境冒烟测试"
	@echo "make test-exact 纯函数单测（只依赖 numpy，秒级：告警措辞 / 盒判定）"
	@echo "make test-config 配置单测（配置文件 == 代码常量 / 解析器 / 优先级）"
	@echo "make test-vr    VR 数据源单测（话题 / 端点 / 队列纪律，装了 eclipse-zenoh 再跑真往返）"
	@echo "make test       运行全部测试"
	@echo ""
	@echo "全部可选项: config/teleop.yaml（含单位与说明，启动脚本与服务端都读它）"
	@echo "启动参数总表: ./launch/run_teleop_webxr.py --help"
	@echo "例:           make teleop ARGS=\"--dual --viewer rviz\""
	@echo "              make teleop ARGS=\"--ik exact --deadband-scale 0\""
	@echo "              make teleop ARGS=\"--config /path/to/other.yaml\""
	@echo "当前解释器: $(if $(PY_ARM),$(PY_ARM),未找到——先跑 make setup-env)"
	@echo "换一个:     make PY_ARM=/path/to/python <target>"

# 解释器不存在时给出指名道姓的报错，别让 make 报一句莫名其妙的 command not found
check-py:
	@test -x "$(PY_ARM)" || { \
	    echo "找不到 metavr-arm 环境的 python：$(PY_ARM)"; \
	    echo "先跑 bash environments/setup.sh，或指定：make PY_ARM=/path/to/python <target>"; \
	    exit 1; \
	}

setup-env:
	bash environments/setup.sh

# 一键启动。ARGS 直通启动脚本（参数总表见 ./launch/run_teleop_webxr.py --help）
teleop:
	./launch/run_teleop_webxr.py $(ARGS)

stop:
	./launch/stop_teleop.sh

smoke: check-py
	PYTHONPATH=src LITEARM_NO_VIEWER=1 $(PY_ARM) tests/test_teleop_sim.py

# 纯函数单测：不 import pybullet，秒级。检验「持位告警怎么措辞」与「盒是哪一半给的」。
# 清空 PYTHONPATH：本机 ROS 路径会污染插件加载。
test-exact: check-py
	PYTHONPATH=src $(PY_ARM) tests/test_exact_ik.py

# 配置单测：配置文件里的出厂值必须与代码常量逐位相同 + 解析器 + 优先级。
test-config: check-py
	PYTHONPATH=src $(PY_ARM) tests/test_config.py

# VR 数据源单测：只用标准库（话题拼法 / 端点解析 / 队列纪律）。装了 eclipse-zenoh
# 还会跑一遍真 pub/sub 往返，没装就跳过那一步并打印原因。
test-vr: check-py
	PYTHONPATH=src $(PY_ARM) tests/test_vr_source.py

# Reproduces fast wrist rotation with the STM32 server's velocity,
# acceleration, and hardware-limit behavior, then checks the feedback protocol.
test-wrist-slew: check-py
	PYTHONPATH=src $(PY_ARM) -m unittest -v tests.test_anchored_tracking.AnchoredTrackingTests.test_downstream_wrist_slew_is_not_misclassified_as_execution_fault
	PYTHONPATH=src $(PY_ARM) -m unittest -v tests.test_real_arm_client.FeedbackTests.test_measured_feedback_and_expiry
	PYTHONPATH=src $(PY_ARM) -m unittest -v tests.test_control_bridge.ControlBridgeTest.test_downstream_applied_command_is_forwarded_to_execution_monitor
	cd ../../litearm-vr-follow && $(PY_ARM) -m unittest -v test_measured_state.HoldTests.test_stm32_state_reports_post_slew_command

test-tracking: check-py
	PYTHONPATH=src $(PY_ARM) -m unittest discover -s tests -p 'test_pose_target.py'
	PYTHONPATH=src $(PY_ARM) -m unittest discover -s tests -p 'test_control_bridge.py'
	PYTHONPATH=src $(PY_ARM) -m unittest discover -s tests -p 'test_real_arm_client.py'

test: check-py test-exact test-config test-vr test-tracking
	# 清空 PYTHONPATH：本机 ROS 路径会污染 pytest 插件加载
	PYTHONPATH=src LITEARM_NO_VIEWER=1 $(PY_ARM) tests/test_teleop_sim.py
