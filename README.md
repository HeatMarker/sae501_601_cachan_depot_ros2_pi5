# Projet ROS2 - Architecture & Contrôle

Ce dépôt contient l'ensemble des packages ROS2 pour le contrôle du robot, incluant la communication (Bridge), la téléopération et la gestion du LiDAR.

> ⚠️ **Note importante sur l'état du projet**
>
> La partie matérielle (hardware) étant désormais terminée et validée, **ce dépôt est en cours de restructuration majeure.**
>
> Bien que le code actuel soit fonctionnel, la majorité de la logique et de l'architecture logicielle sera **réécrite prochainement** pour s'adapter parfaitement aux spécificités finales du hardware.

## Modules fonctionnels

Les éléments suivants sont actuellement disponibles et opérationnels :

* **ROS2 Bridge** : Assure l'interface de communication entre le PC (High-level) et le microcontrôleur (Low-level / STM32).
* **Teleop** : Noeuds permettant le contrôle manuel du robot (manette/clavier).
* **LiDAR** : Acquisition et publication des données du capteur laser.

## Utilisation (Version actuelle)

Prérequis : Environnement ROS2 installé et configuré.

1.  Cloner ce dépôt dans le dossier `src/` de l'espace de travail (workspace).
2.  Compiler les packages :
    ```bash
    colcon build --symlink-install
    ```
3.  Sourcer l'environnement :
    ```bash
    source install/setup.bash
    ```
