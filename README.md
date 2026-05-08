# Sequential vs Static Fault Detection for USV Swarms

Implementation for the paper: **"Learning Faults in Time: Sequential Behavioural Modelling for Complex Fault Detection in Multi-Robot Systems"**

> Faisal Firas Mazloum, David Portugal, Micael S. Couceiro  
> *Frontiers in Robotics and AI*, 2026 (under review)

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.20082838.svg)](https://doi.org/10.5281/zenodo.20082838)

---

## 📺 Demonstrations

#### 🔹 Progressive Actuator Degradation Detection
https://github.com/user-attachments/assets/7771c56a-42a1-4d3e-99e2-2dfcfdc89d7a

#### 🔹 Intermittent Fault Detection
https://github.com/user-attachments/assets/e65c580a-da22-44be-87c8-0bbafb7e7049

---

## Repository Structure
sequential-vs-static-fault-detection-swarm/
├── ros2_ws/        # ROS2 workspace including fault detection nodes (LSTM and GBDT)
├── unity_sim/      # Unity aquatic simulator for the REMORA USV swarm
└── ROS2swarm/      # ROS2 swarm behaviour package

---

## Dependencies

**ROS2:** Jazzy  
**Unity:** 2022.3.62f1 LTS  
**Python:** 3.12.3  

| Package | Version |
|---------|---------|
| tensorflow | 2.20.0 |
| scikit-learn | 1.3.2 |
| joblib | 1.5.2 |
| numpy | 1.26.4 |
| pandas | 2.3.3 |
| psutil | 7.1.0 |

---

## 🇪🇺 Acknowledgments

This project has received funding from the European Union's Horizon Europe research and innovation programme under the Marie Skłodowska-Curie Actions (MSCA) Doctoral Network, under grant agreement No **101169330**. Views and opinions expressed are those of the author(s) only and do not necessarily reflect those of the European Union or the European Research Executive Agency (REA).

---

## Citation

BibTeX will be added upon acceptance.

---

© 2026 Faisal Mazloum — AIGreenBots Project
