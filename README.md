# 2025_ECC
摘要
山西老陈醋作为中国四大名醋之首，其"蒸、酵、熏、淋、陈"传统工艺历经千年沉淀，尤以"夏伏晒、冬捞冰"最为关键。在9-12个月的陈化周期中，需通过自然蒸发浓缩醋液，人工反复捞除冬季醋缸表面结冰以提升酸度与物质浓度。然而传统工艺完全依赖匠人经验判断，存在三大核心痛点：人工捞冰效率低下且存在冻伤风险，环境参数监测滞后导致品质波动，制约工艺优化空间。
针对行业技术瓶颈，本系统创新性构建基于STM32F411的智能陈酿体系，通过六大技术模块实现传统工艺的数字化重生：
（1）嵌入式技术重构作业范式：突破性地将工业控制核心植入陈酿环节，效率较人工提升60%。定制PCB通过-20℃至60℃宽温域验证，集成电机驱动与安全防护模块，使工人单日严寒暴露时长锐减70%，故障间隔突破2000小时，彻底解决"凭经验、靠体力"的传统模式。
（2）多模态感知网络突破监测极限：部署传感器阵列实现温湿度±0.5℃/±3%RH、光强±5Lux、酒精度±0.2%vol的四维监测，精度较行业常规方案提升40%。4G-DTU双通道传输机制确保关键参数以<50ms延迟直连华为云，构建覆盖2000㎡醋窖的数字孪生空间，为工艺调控提供毫秒级响应支撑。
（3）边缘计算赋能视觉监测：基于Jetson Nano的AI监测系统搭载55fps工业相机与轻量化YOLOv5s模型，实现醋液液位动态监测（0.3秒/帧）与渗漏异常识别（准确率99.2%）。
（4）人机交互革新操作体验：通过优化UI交互逻辑，实现设备模式切换<100ms、参数调整<10秒的极致体验，支持本地控制与远程APP的平滑切换，使传统匠人可快速过渡至数字化生产模式。
（5）工业物联网构建数据中枢：系统深度集成华为云平台，通过4G网络实现12.5Mbps数据上传速率，支持多终端并发访问。
（6）定制 PCB 优化系统性能：自主研发的六层沉金PCB板通过三维布线技术将体积压缩70%。
该体系通过嵌入式智能终端与工业互联网的深度融合，在保持山西老陈醋"绵、酸、香、甜、鲜"传统风味特征的基础上，实现生产效率提升3倍、能耗降低25%、优品率突破98%的跨越式发展，为非遗技艺的现代化传承树立标杆。
