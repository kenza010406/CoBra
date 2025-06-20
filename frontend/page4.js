// page4.js - Mode 2
// Gestion de la page Mode 2 du système CoBra (visualisation prostate)

// Fonction d'initialisation de la page Mode 2
function initMode2Page() {
    console.log('Initialisation de la page Mode 2');
    
    // Créer l'élément de contenu principal
    const mode2Content = document.createElement('div');
    mode2Content.id = 'mode2';
    mode2Content.className = 'page';
    mode2Content.style.display = 'none'; // Masqué par défaut
    
    // Créer le panneau de contrôle
    const controlPanel = document.createElement('div');
    controlPanel.className = 'control-panel';
    
    // Panneau de gauche (Contrôles)
    const leftPanel = createJoystickPanel();
    
    // Panneau de droite (Visualisation prostate)
    const rightPanel = createProstateVisualizationPanel();
    
    // Assembler les panneaux dans le panneau de contrôle
    controlPanel.appendChild(leftPanel);
    controlPanel.appendChild(rightPanel);
    
    // Ajouter le panneau de contrôle à la page
    mode2Content.appendChild(controlPanel);
    
    // Ajouter la page au conteneur principal
    document.querySelector('.content').appendChild(mode2Content);
    
    // Enregistrer les gestionnaires d'événements spécifiques à cette page
    setupMode2EventListeners();
    
    return mode2Content;
}

// Fonction pour créer le panneau de joystick et contrôles (similaire au Mode 1)
function createJoystickPanel() {
    const panel = document.createElement('div');
    panel.className = 'panel';
    
    // Titre du panneau
    const title = document.createElement('h3');
    title.className = 'panel-title';
    title.textContent = 'Contrôle Joystick';
    
    // Conteneur des joysticks
    const joystickContainer = document.createElement('div');
    joystickContainer.className = 'joystick-container';
    
    // Joystick directionnel (gauche)
    const joystick1 = document.createElement('div');
    joystick1.className = 'joystick';
    joystick1.id = 'joystick1Mode2';
    
    // Flèches pour le joystick 1
    ['up', 'down', 'left', 'right'].forEach(direction => {
        const arrow = document.createElement('div');
        arrow.className = `arrow arrow-${direction}`;
        joystick1.appendChild(arrow);
    });
    
    // Joystick vertical (droite)
    const joystick2 = document.createElement('div');
    joystick2.className = 'joystick';
    joystick2.id = 'joystick2Mode2';
    joystick2.style.width = '80px'; // Plus petit
    
    // Flèches pour le joystick 2
    ['up', 'down'].forEach(direction => {
        const arrow = document.createElement('div');
        arrow.className = `arrow arrow-${direction}`;
        joystick2.appendChild(arrow);
    });
    
    // Ajouter les joysticks au conteneur
    joystickContainer.appendChild(joystick1);
    joystickContainer.appendChild(joystick2);
    
    // Conteneur de curseur de vitesse
    const sliderContainer = document.createElement('div');
    sliderContainer.className = 'slider-container';
    
    // Étiquette du curseur
    const sliderLabel = document.createElement('div');
    sliderLabel.className = 'slider-label';
    sliderLabel.textContent = 'Vitesse';
    
    // Curseur de vitesse
    const slider = document.createElement('input');
    slider.type = 'range';
    slider.min = '0';
    slider.max = '100';
    slider.value = '50';
    slider.className = 'slider';
    slider.id = 'speedSliderMode2';
    
    // Ajouter l'étiquette et le curseur au conteneur
    sliderContainer.appendChild(sliderLabel);
    sliderContainer.appendChild(slider);
    
    // Indicateur de connexion CAN
    const connectionStatus = document.createElement('div');
    connectionStatus.className = 'connection-status';
    
    // Étiquette de statut
    const statusLabel = document.createElement('div');
    statusLabel.className = 'status-label';
    statusLabel.textContent = 'Connexion CAN :';
    
    // Indicateur visuel
    const statusIndicator = document.createElement('div');
    statusIndicator.className = 'status-indicator';
    statusIndicator.id = 'canStatusMode2';
    
    // Assembler l'indicateur de connexion
    connectionStatus.appendChild(statusLabel);
    connectionStatus.appendChild(statusIndicator);
    
    // Assembler tous les éléments dans le panneau
    panel.appendChild(title);
    panel.appendChild(joystickContainer);
    panel.appendChild(sliderContainer);
    panel.appendChild(connectionStatus);
    
    return panel;
}

// Fonction pour créer le panneau de visualisation de la prostate
function createProstateVisualizationPanel() {
    const panel = document.createElement('div');
    panel.className = 'panel';
    
    // Titre du panneau
    const title = document.createElement('h3');
    title.className = 'panel-title';
    title.textContent = 'Visualisation de l\'aiguille';
    
    // Conteneur de visualisation de la prostate
    const prostateVisual = document.createElement('div');
    prostateVisual.className = 'prostate-visual';
    
    // Titre de visualisation
    const prostateTitle = document.createElement('div');
    prostateTitle.className = 'prostate-title';
    prostateTitle.textContent = 'Visualisation 3D Prostate';
    
    prostateVisual.appendChild(prostateTitle);
    
    // Affichage des coordonnées (comme dans le Mode 1)
    const coordinates = document.createElement('div');
    coordinates.className = 'coordinates';
    
    // Coordonnée X
    const coordX = document.createElement('div');
    coordX.className = 'coordinate';
    coordX.id = 'coordXMode2';
    coordX.innerHTML = 'X: <span class="positive">+28</span>';
    
    // Coordonnée Y
    const coordY = document.createElement('div');
    coordY.className = 'coordinate';
    coordY.id = 'coordYMode2';
    coordY.innerHTML = 'Y: <span class="negative">-16</span>';
    
    // Coordonnée Z
    const coordZ = document.createElement('div');
    coordZ.className = 'coordinate';
    coordZ.id = 'coordZMode2';
    coordZ.innerHTML = 'Z: <span class="positive">+5</span>';
    
    // Ajouter les coordonnées au conteneur
    coordinates.appendChild(coordX);
    coordinates.appendChild(coordY);
    coordinates.appendChild(coordZ);
    
    // Boutons d'action
    const actionButtons = document.createElement('div');
    actionButtons.className = 'action-buttons';
    
    // Bouton Réinitialiser
    const resetButton = document.createElement('button');
    resetButton.className = 'action-btn reset-btn';
    resetButton.textContent = 'Réinitialiser';
    resetButton.id = 'resetButtonMode2';
    
    // Bouton d'arrêt d'urgence
    const emergencyButton = document.createElement('button');
    emergencyButton.className = 'action-btn emergency-btn';
    emergencyButton.textContent = 'Arrêt d\'urgence';
    emergencyButton.id = 'emergencyButtonMode2';
    
    // Ajouter les boutons au conteneur
    actionButtons.appendChild(resetButton);
    actionButtons.appendChild(emergencyButton);
    
    // Assembler tous les éléments dans le panneau
    panel.appendChild(title);
    panel.appendChild(prostateVisual);
    panel.appendChild(coordinates);
    panel.appendChild(actionButtons);
    
    return panel;
}

// Configuration des écouteurs d'événements pour la page Mode 2
function setupMode2EventListeners() {
    console.log('Configuration des écouteurs d\'événements pour la page Mode 2');
    
    // Gestionnaire pour le curseur de vitesse
    const speedSlider = document.getElementById('speedSliderMode2');
    if (speedSlider) {
        speedSlider.addEventListener('input', function() {
            const value = this.value;
            
            // Mettre à jour l'affichage du curseur
            this.style.background = `linear-gradient(to right, #4caf50 ${value}%, #e0e0e0 ${value}%)`;
            
            console.log('Vitesse ajustée à:', value);
        });
    }
    
    // Gestionnaires pour les boutons
    const resetButton = document.getElementById('resetButtonMode2');
    if (resetButton) {
        resetButton.addEventListener('click', function() {
            console.log('Réinitialisation du système (Mode 2)');
            
            // Réinitialiser les coordonnées
            updateNeedleCoordinatesMode2(0, 0, 0);
        });
    }
    
    const emergencyButton = document.getElementById('emergencyButtonMode2');
    if (emergencyButton) {
        emergencyButton.addEventListener('click', function() {
            console.log('ARRÊT D\'URGENCE ACTIVÉ (Mode 2)');
            alert('ARRÊT D\'URGENCE : Tous les mouvements ont été arrêtés.');
        });
    }
    
    // Simulation de l'interactivité des joysticks
    setupJoystickInteractionsMode2();
}

// Fonction pour configurer l'interactivité des joysticks (Mode 2)
function setupJoystickInteractionsMode2() {
    // Le joystick 1 (XY)
    const joystick1 = document.getElementById('joystick1Mode2');
    if (joystick1) {
        // Flèches du joystick 1
        const arrows = joystick1.querySelectorAll('.arrow');
        arrows.forEach(arrow => {
            arrow.addEventListener('mousedown', function() {
                this.style.opacity = '0.5';
                
                // Identifier la direction et mettre à jour les coordonnées
                const direction = this.className.split('arrow-')[1];
                updateJoystickMovementMode2(direction);
            });
            
            arrow.addEventListener('mouseup', function() {
                this.style.opacity = '1';
            });
            
            arrow.addEventListener('mouseleave', function() {
                this.style.opacity = '1';
            });
        });
    }
    
    // Le joystick 2 (Z)
    const joystick2 = document.getElementById('joystick2Mode2');
    if (joystick2) {
        // Flèches du joystick 2
        const arrows = joystick2.querySelectorAll('.arrow');
        arrows.forEach(arrow => {
            arrow.addEventListener('mousedown', function() {
                this.style.opacity = '0.5';
                
                // Identifier la direction et mettre à jour les coordonnées
                const direction = this.className.split('arrow-')[1];
                updateJoystickZMovementMode2(direction);
            });
            
            arrow.addEventListener('mouseup', function() {
                this.style.opacity = '1';
            });
            
            arrow.addEventListener('mouseleave', function() {
                this.style.opacity = '1';
            });
        });
    }
}

// Fonction pour mettre à jour le mouvement XY en fonction du joystick 1 (Mode 2)
function updateJoystickMovementMode2(direction) {
    console.log('Mouvement du joystick 1 (Mode 2):', direction);
    
    // Récupérer les coordonnées actuelles
    const xElement = document.querySelector('#coordXMode2 span');
    const yElement = document.querySelector('#coordYMode2 span');
    
    // Extraire les valeurs numériques (retirer le signe +/-)
    let x = parseInt(xElement.textContent);
    let y = parseInt(yElement.textContent);
    
    // Mise à jour en fonction de la direction
    const speed = parseInt(document.getElementById('speedSliderMode2').value) / 10;
    
    switch(direction) {
        case 'up':
            y += speed;
            break;
        case 'down':
            y -= speed;
            break;
        case 'left':
            x -= speed;
            break;
        case 'right':
            x += speed;
            break;
    }
    
    // Mettre à jour les coordonnées affichées
    updateNeedleCoordinatesMode2(x, y, null);
}

// Fonction pour mettre à jour le mouvement Z en fonction du joystick 2 (Mode 2)
function updateJoystickZMovementMode2(direction) {
    console.log('Mouvement du joystick 2 (Mode 2):', direction);
    
    // Récupérer la coordonnée Z actuelle
    const zElement = document.querySelector('#coordZMode2 span');
    
    // Extraire la valeur numérique (retirer le signe +/-)
    let z = parseInt(zElement.textContent);
    
    // Mise à jour en fonction de la direction
    const speed = parseInt(document.getElementById('speedSliderMode2').value) / 10;
    
    switch(direction) {
        case 'up':
            z += speed;
            break;
        case 'down':
            z -= speed;
            break;
    }
    
    // Mettre à jour la coordonnée Z affichée
    updateNeedleCoordinatesMode2(null, null, z);
}

// Fonction pour mettre à jour les coordonnées de l'aiguille (Mode 2)
function updateNeedleCoordinatesMode2(x, y, z) {
    // Mettre à jour X si fourni
    if (x !== null) {
        const xElement = document.querySelector('#coordXMode2 span');
        xElement.textContent = (x >= 0 ? '+' : '') + x;
        xElement.className = x >= 0 ? 'positive' : 'negative';
    }
    
    // Mettre à jour Y si fourni
    if (y !== null) {
        const yElement = document.querySelector('#coordYMode2 span');
        yElement.textContent = (y >= 0 ? '+' : '') + y;
        yElement.className = y >= 0 ? 'positive' : 'negative';
    }
    
    // Mettre à jour Z si fourni
    if (z !== null) {
        const zElement = document.querySelector('#coordZMode2 span');
        zElement.textContent = (z >= 0 ? '+' : '') + z;
        zElement.className = z >= 0 ? 'positive' : 'negative';
    }
    
    // À faire: Mettre à jour la visualisation 3D de la prostate (à implémenter)
    // updateProstateVisualization(x, y, z);
}

// Fonction pour nettoyer la page lors du changement de page
function cleanupMode2Page() {
    console.log('Nettoyage de la page Mode 2');
    
    // Supprimer les écouteurs d'événements pour éviter les fuites de mémoire
    
    // Réinitialiser les joysticks
    const joystick1 = document.getElementById('joystick1Mode2');
    if (joystick1) {
        const arrows = joystick1.querySelectorAll('.arrow');
        arrows.forEach(arrow => {
            arrow.removeEventListener('mousedown', null);
            arrow.removeEventListener('mouseup', null);
            arrow.removeEventListener('mouseleave', null);
        });
    }
    
    const joystick2 = document.getElementById('joystick2Mode2');
    if (joystick2) {
        const arrows = joystick2.querySelectorAll('.arrow');
        arrows.forEach(arrow => {
            arrow.removeEventListener('mousedown', null);
            arrow.removeEventListener('mouseup', null);
            arrow.removeEventListener('mouseleave', null);
        });
    }
    
    // Réinitialiser le curseur de vitesse
    const speedSlider = document.getElementById('speedSliderMode2');
    if (speedSlider) {
        speedSlider.removeEventListener('input', null);
    }
    
    // Réinitialiser les boutons d'action
    const resetButton = document.getElementById('resetButtonMode2');
    if (resetButton) {
        resetButton.removeEventListener('click', null);
    }
    
    const emergencyButton = document.getElementById('emergencyButtonMode2');
    if (emergencyButton) {
        emergencyButton.removeEventListener('click', null);
    }
}

// Exportation des fonctions pour utilisation dans le module principal
export {
    initMode2Page,
    updateNeedleCoordinatesMode2,
    cleanupMode2Page
};
