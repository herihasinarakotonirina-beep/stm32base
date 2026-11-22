# STM32 - Nœud d'Actionnement Robotique (FreeRTOS & Bus CAN)

> 🚧 **Statut du projet : En cours de développement** (Projet de Semestre 3 — IUT de Cachan, dans le cadre de la **Coupe de France de Robotique**).

Firmware embarqué développé en **C/C++ sur STM32** (VS Code / PlatformIO) sous **FreeRTOS**, dédié au pilotage temps réel d'actionneurs robotiques (servomoteurs intelligents et moteur pas à pas) et à la télémétrie sur **Bus CAN**.

## Architecture logicielle (FreeRTOS)
Le système repose sur une architecture multi-tâches synchronisée par des files de messages (`Queues`) et protégée par des `Mutex` (accès concurrents au bus UART et au moniteur série) :

- **`tacheCAN_RX` & `tacheCAN_TX` :** Réception et émission des trames sur le Bus CAN (`1 Mbit/s`) sans blocage.
- **`passerelle_CAN` :** Routage et décodage des ordres reçus :
  - `ID 0x44` : Demande de position du servomoteur et renvoi de l'angle sur le bus (`ID 0x100`).
  - `ID 0x10` : Consigne de déplacement (en pas) transmise à la tâche moteur pas à pas.
- **`gestion_servo` :** Pilotage en UART (`Serial1`) des servomoteurs **Herkulex** (contrôle en position/vitesse, coupure de couple à l'arrivée et remontée des codes d'erreur sur `ID 0x20`).
- **`gestion_moteur` *(en cours de mise au point)* :** Commande cadencée à période fixe (`1 ms` via `vTaskDelayUntil`) d'un moteur pas à pas avec profil d'accélération (`AccelStepper`).

## État d'avancement
- [x] Communication Bus CAN (`1 Mbit/s`) et passerelle de routage multi-tâches
- [x] Commande, lecture de position et diagnostic des servomoteurs Herkulex (UART)
- [ ] Mise au point et validation du pilotage du moteur pas à pas (`AccelStepper`)
- [ ] Intégration simultanée des tâches `gestion_servo` et `gestion_moteur`
- [ ] Ajout de la sécurité matérielle par Watchdog

## Matériel et bibliothèques
- **Microcontrôleur :** STM32 (Framework Arduino + `STM32FreeRTOS`)
- **Actionneurs :** Servomoteurs Herkulex DRS (`HerkulexServo`) + Moteur pas à pas (`AccelStepper`)
- **Communication :** Bus CAN (`STM32_CAN`, broches `PA11`/`PA12`) & UART (`PB6`/`PB7` à `115200 bauds`)
