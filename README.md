# 🧭 Mesa Labirinto Controlada por Joystick  
### Projeto Final – Sistemas Embarcados – 2025.2
### Equipe: Aryel Souza,Kevin Ryan,Thiago Barbosa,Plinio

Este projeto implementa uma **mesa com labirinto controlada por joystick**, utilizando **ESP32**, **MPU6050**, **servomotores**, **InfluxDB** e **Grafana**.  
O objetivo é criar um sistema físico capaz de inclinar a mesa para movimentar uma esfera metálica e, ao mesmo tempo, apresentar um **gêmeo digital** da mesa no computador.

---

## 📌 Funcionalidades Principais

### **Fase 1 – Controle Local da Mesa**
- Leitura dos eixos X/Y do joystick.
- Conversão para PWM aplicado aos servomotores.
- Movimentação suave da mesa.
- Estrutura de software baseada em **FreeRTOS**:
  - Task 1 → Leitura do joystick  
  - Task 2 → Controle dos servos  
  - Task 3 → Logs / debug via serial  

---

### **Fase 2 – Leitura da Orientação (MPU6050)**
- Leitura via I²C de acelerômetro e giroscópio.
- Cálculo dos ângulos **pitch** e **roll**.
- Envio periódico dos dados pela serial (JSON).
- Task exclusiva para o MPU6050.

---

### **Fase 3 – Gêmeo Digital (Grafana + InfluxDB)**
- Uso do InfluxDB como *time series database*.
- Script Python lê a serial do ESP32 e envia os dados ao InfluxDB.
- Dashboard em Grafana mostrando:
  - Pitch em tempo real  
  - Roll em tempo real  
  - Representação gráfica da mesa (gêmeo digital)

---

## 🛠️ Tecnologias Utilizadas

### **Hardware**
- ESP32
- Joystick analógico
- 2x Servomotores 90G
- MPU6050
- LED de status
- Estrutura mecânica da mesa

### **Software**
- ESP-IDF (FreeRTOS)
- Python 3
- InfluxDB 2.x
- Grafana OSS
- Dashboard Grafana com Canvas

---

