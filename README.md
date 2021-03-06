# GpsN2KWifiGateaway

## Présentation

L'objectif initial de ce projet etait de disposer d'un GPS NMEA 2000 ayant une fréquence de rafraichissement d'au moins 10Hz pour l'intégrer dans notre Archambault A31 (https://epsilon3dsailingteam.fr/).

Il semble que les pilotes automatiques intègrent les données GPS pour effectuer des corrections de trajectoire.

Les GPS Ublox M8N (assez peu couteux) permettent, apèrs configuration, d'obtenir ces taux de rafraichissement en étant mono constellation.
Le nouveelles puces M9N permettent d'obtenir un taux de rafraichissement de 25Hz en exploitant les 4 constellations (GPS, Glonass, Galiléo et Baidoo).


## Projet

Le projet est constitué : 
- Une partie électronique constituée :
  - D'une puce ESP 32 
  - D'un puce GPS M9N, dans la première version nous utilisons une carte Sparkfun (https://www.sparkfun.com/products/15712)
- Un boitier à imprimer en 3D (en cours)
- Le code source du projet

Le code source peut être ouvert avec l'IDE Arduino et le projet électronique avec kiCAD

## Librairies externes utilisées

Le code source utilise ces librairies, sous licence MIT elles aussi :
- Plusieurs librairies de, https://github.com/ttlappalainen : NMEA0183, NMEA2000, NMEA2000_esp32
- https://github.com/ronzeiller/NMEA0183-AIS
- https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library

## Fonctions implémentées

Actuellement le projet permet :
- D'injecter sur le réseau NMEA2000 les données de la puce GPS a la fréquence de 20Hz (25Hz possible, a voir l'intérêt)
- Se connecter à un réseau Wifi existant
- Creer un point d'accès Wifi
- Lire les données du réseau NMEA2000, les convertir en MNEA0183 et les router vers :
-- 3 ports série 
-- Wifi en TCP et en UDP

## RoadMap
- Page Web de configuration visualisation
  - Ajout du serveur de configuration Wifi pour configuration sur un routeur externe
  - Ajout de fonction AJAX/JSON pour rechargement de la page de statut
  - Mise en une seule page pour intégrer la fonction de MAJ du firmware
  - Ajout d'option pour activer certaines redirections vers les ports et le reseau (a stocker en NVS)
  - Mise en page et logo
  - Configuration des ports de sortie serie : Msg NMEA, vitesse, activation...
  - Configuration du SSID externe + PWD
