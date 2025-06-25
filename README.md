# CoBra
RPI4:
/home/ubuntu/
│
├── cobra_ws/                     # Workspace ROS2
│   ├── src/
│   │   ├── cobra_msgs/          # Package de messages
│   │   │   ├── msg/
│   │   │   │   ├── NeedlePosition.msg
│   │   │   │   ├── MotorStatus.msg
│   │   │   │   └── SystemStatus.msg
│   │   │   ├── CMakeLists.txt
│   │   │   └── package.xml
│   │   └── cobra_control1/       # Package de contrôle
│   │       ├── cobra_control1/
│   │       │   └── ui_controller.py
│   │       ├── launch/
│   │       │   └── cobra_ui.launch.py
│   │       ├── setup.py
│   │       └── package.xml
│   │
│   ├── build/                   # Généré à la compilation
│   ├── install/                 # Généré à la compilation
│   └── log/                     # Généré à la compilation
│
└── cobra_web/                   # Serveur web
    ├── server.js                # Serveur Express
    └── public/                  # Fichiers de l'interface
        ├── index.html           # Votre HTML
        ├── style.css            # Votre CSS
        ├── main.js              # Vos fichiers JS
        ├── page1.js
        ├── page2.js
        ├── page3.js
        ├── page4.js
        ├── page5.js
        ├── page6.js
        ├── page7.js
        ├── script.js
        ├── robot-medical.png    # Vos images
        ├── roslib.min.js        # Bibliothèque roslibjs
        └── ros_bridge.js        # Script de connexion ROS




RPI5:
/home/ubuntu/
│
└── cobra2_ws/                     # Workspace ROS2
    ├── src/
    │   ├── cobra_msgs/          # Package de messages (identique au RPI4)
    │   │   ├── msg/
    │   │   │   ├── NeedlePosition.msg
    │   │   │   ├── MotorStatus.msg
    │   │   │   └── SystemStatus.msg
    │   │   ├── CMakeLists.txt
    │   │   └── package.xml
    │   │
    │   └── cobra_motor_control_py/ # Package de contrôle moteur
    │       ├── cobra_motor_control_py/
    │       │   └── motor_controller.py
    │       │   └── motor_can_driver.py
    │       ├── launch/
    │       │   └── cobra_motor.launch.py
    │       ├── setup.py
    │       └── package.xml
    │
    ├── build/                   # Généré à la compilation
    ├── install/                 # Généré à la compilation
    └── log/                     # Généré à la compilation
