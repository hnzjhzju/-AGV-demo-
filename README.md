# -AGV-demo-
【个人试验项目】
设计并实现智慧港口AGV无人运输调度解决方案，基于多车协同调度算法与数字孪生仿真系统，完成AGV路径规划、任务分配与交通控制机制设计，并通过可视化仿真验证系统性能。
【项目架构】
agv-scheduling-demo/
├── backend/
│   ├── scheduler.py
│   ├── path_planning.py
│
├── frontend/
│   ├── index.html
│   ├── simulation.js
│
├── docs/
│   ├── solution.md
│   ├── architecture.png
│
├── demo_video.mp4
【项目亮点】
1.多车调度策略：规则 + 启发式优化（最短路径 + 负载均衡）
2.路径规划：A* + 动态重规划
3.死锁避免：基于路径占用的资源锁机制
4.仿真系统：支持10~50台AGV协同运行
【成果指标/ROI】
仿真环境下AGV调度效率提升约 15%
路径冲突减少约 30%
支持多车无死锁运行
