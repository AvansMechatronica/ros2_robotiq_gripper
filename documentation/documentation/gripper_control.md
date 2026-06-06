# Robotiq Gripper Besturingscripts

Deze map bevat Python-scripts voor het besturen van de Robotiq gripper in ROS2.

## Starten van de controller
Alvorens de gripper is te bedienen dient de controller te zijn gestart. Dit kan als volgt:
```bash
ros2 launch my_robotiq_controller robotiq_controller.launch.py 
```

:::{note}
Op Ubuntu wordt de groep dialout gebruikt om toegang te geven tot seriële apparaten zoals /dev/ttyUSB0, /dev/ttyACM0 en vergelijkbare USB-seriële adapters.

Controleer of je gebruiker al in de groep zit
```bash
groups
```

Of specifiek:

```bash
id -nG $USER
```

Als dialout niet in de lijst staat, voeg jezelf toe:

```bash
sudo usermod -aG dialout $USER
```
:::

## 1. GUI-besturing (`gripper_gui.py`)

Een grafische gebruikersinterface voor gripper-besturing en monitoring.

```bash
ros2 run robotiq_app gripper_gui.py
```


## 2. Commandoregel-besturing (`control_gripper`)

Gebruik de volgende commando's om de gripper te bedienen:

### Gripper openen
```bash
ros2 run robotiq_app control_gripper --open
```

### Gripper sluiten
```bash
ros2 run robotiq_app control_gripper --close
```

### Naar specifieke positie verplaatsen (0.0 = gesloten, 0.085 = open voor 2F-85)
```bash
ros2 run robotiq_app control_gripper --position 0.04
```

### Positie instellen met aangepaste kracht (in Newton)
```bash
ros2 run robotiq_app control_gripper --position 0.02 --effort 100.0
```

### Activeren en vervolgens openen (meerdere commando's)
```bash
ros2 run robotiq_app control_gripper --activate --open
```

## 3. Directe gripper besturing via ROS2-topics
Je kunt ook direct berichten publiceren op de ROS2-topics om de gripper te besturen:

### Nieuw topic voor directe positie-aansturing
De controller subscribe nu op:

- Topic: `/robotiq_gripper_controller/set_joint_state`
- Type: `std_msgs/msg/Float64`

De waarde in `data` is de gewenste gripperpositie in meter (voor 2F-85):

- `0.0` = volledig gesloten
- `0.085` = volledig open

Waarden buiten dit bereik worden in de node automatisch begrensd.

### Voorbeelden publiceren

Volledig openen:

```bash
ros2 topic pub --once /robotiq_gripper_controller/set_joint_state std_msgs/msg/Float64 "{data: 0.085}"
```

Volledig sluiten:

```bash
ros2 topic pub --once /robotiq_gripper_controller/set_joint_state std_msgs/msg/Float64 "{data: 0.0}"
```

Halverwege openen:

```bash
ros2 topic pub --once /robotiq_gripper_controller/set_joint_state std_msgs/msg/Float64 "{data: 0.04}"
```

Controleer eventueel eerst of de subscriber actief is:

```bash
ros2 topic info /robotiq_gripper_controller/set_joint_state
```

