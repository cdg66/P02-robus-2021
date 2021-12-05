#ifndef _MANUEL_H__
#define _MANUEL_H__

// define

// public variables
bool isModeMan;

// public function
//fonction pour le callback, s'occupe de lire les données reçu et ajuste la vitesse des moteur
void lireBluetooth();

//fonction qui update dans l'app si le robot voit une mine ou non
void mineStatus(bool status);


void manuel_init();
void manuelStart();
void manuelStop();

//fonctions qui devront être changer pour appeler les bonnes fonctions dans le main
void dropDrapeau(int drapeau);

#endif
