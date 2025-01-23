# SandPanda

Documentation du code SandPanda dans son état.

**`AffinityCache`** 
    Permet de fixer une priorité de mémoire cache pour éviter la perte de la mémoire cache lors d'un changement de processeur.

**`Body`** représente une particule composée de sphères (position, vitesse, ...)

**` BodySpecie`** définition d'une particule composées de sphères (position des sphères, matrice d'inertie, ...)

**`Compaction`** méthode pour produire la compaction par alternance de secousse (vibration du récipient) et de relaxation (jusqu'au repos)  

**`Cone`** représente un cône comme container (in) ou obstacle (out) 

**`Contact`** représente un contact avec toutes les données associées

**`ContactDetection`** implémente toutes les méthodes de détections de contact pour toutes les combinaisons de solides possibles.

**`Container`** représente un plan, par exemple, par une agglomération de sphères. Ce dernier réagit aux forces subies.

**`Data`** agglomération des paramètres du logiciel, ou configuration.

**`Elbow`** représente un tuyau en forme de coude

**`Elongation`** élongation d'un ressort tangentiel de contact. Il évolue de pas de temps en pas de temps afin de créer une force de friction statique. Dans les autres modèles, la force de friction dépend de la vitesse, elle ne peut donc pas persister lors de l'étude d'un empilement statique.

**`Evolution`** évolution temporelle du système par une boucle de pas de temps. Lors de chaque itération, l'ensemble des étapes nécessaires sont réalisées.

**`Gravity`** représente la gravité. Cette classe permet de faire tourner la gravité. Cette solution est plus simple que de faire tourner tout le système.

**`HollowBall`** représente un container de forme sphérique. Ce dernier peut avoir des degrés de liberté verouillés.

**`main`** Effectue les étapes suivantes :
1. Déclaration des données du programme
2. Chargement et initialisation des données
3. Préparation de la simulation avec et sans OpenMP
4. Appel aux méthodes Evolution, Compaction, PowderPaQ pour les calculs
5. Libérations des données

**`MasterSolid`** Agglomération cohérente de solides pouvant répondre aux forces subies.

**`Move`** implémente la mise à jour des vitesses et des positions avec et sans OpenMP pour tous les types.

**`Option`** gestion des options du programme

**`Periodicity`** implémente les coordonnées des sphères et bodies dans le voisinages d'une paire de plans périodiques.

**`Plan`** représente une paroie rectangulaire 

**`PlanR`** représente une paroie circulaire

**`PowderPaQ`** évolution adaptée à l'expérience d'un appareil PowderPaQ

**`ReadWrite`** Lecture et écriture des données dans des fichiers textes

**`SandPanda`** alternative au main

**`Sinusoid`** modélise une sinusoide pour l'évolution périodique d'une force, d'une vitesse ou d'une position.

**`Solid`** classe mère de tous les solides de type container (Cone, Plan et PlanR) par opposition aux particules.

**`Sphere`** représente une particule de forme sphérique

**`Velocity`** représente les vitesses pour chaque dimension de translation et de rotation sous la forme d'une sinusoïde.



