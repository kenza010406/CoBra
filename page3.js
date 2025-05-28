// page3.js - Mode 1
// Gestion de la page Mode 1 du système CoBra (contrôle grille de coordonnées)

// Fonction d'initialisation de la page Mode 1
function initMode1Page() {
    console.log('Initialisation de la page Mode 1');
    
    // Créer l'élément de contenu principal
    const mode1Content = document.createElement('div');
    mode1Content.id = 'mode1';
    mode1Content.className = 'page';
    mode1Content.style.display = 'none'; // Masqué par défaut
    
    // Créer le panneau de contrôle
    const controlPanel = document.createElement('div');
    controlPanel.className = 'control-panel';
    
    // Panneau de gauche (Contrôles)
    const leftPanel = createJoystickPanel();
    
    // Panneau de droite (Visualisation)
    const rightPanel = createGridVisualizationPanel();
    
    // Assembler les panneaux dans le panneau de contrôle
    controlPanel.appendChild(leftPanel);
    controlPanel.appendChild(rightPanel);
    
    // Ajouter le panneau de contrôle à la page
    mode1Content.appendChild(controlPanel);
    
    // Ajouter la page au conteneur principal
    document.querySelector('.content').appendChild(mode1Content);
    
    // Enregistrer les gestionnaires d'événements spécifiques à cette page
    setupMode1EventListeners();
    
    return mode1Content;
}

// Fonction pour créer le panneau de joystick et contrôles
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
    joystick1.id = 'joystick1';
    
    // Flèches pour le joystick 1
    ['up', 'down', 'left', 'right'].forEach(direction => {
        const arrow = document.createElement('div');
        arrow.className = `arrow arrow-${direction}`;
        joystick1.appendChild(arrow);
    });
    
    // Joystick vertical (droite)
    const joystick2 = document.createElement('div');
    joystick2.className = 'joystick';
    joystick2.id = 'joystick2';
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
    slider.id = 'speedSlider';
    
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
    statusIndicator.id = 'canStatus';
    
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

// Fonction pour créer le panneau de visualisation de la grille
function createGridVisualizationPanel() {
    const panel = document.createElement('div');
    panel.className = 'panel';
    
    // Titre du panneau
    const title = document.createElement('h3');
    title.className = 'panel-title';
    title.textContent = 'Visualisation de l\'aiguille';
    
    // Grille de visualisation
    const grid = document.createElement('div');
    grid.className = 'visualization-grid';
    grid.id = 'visualizationGrid';
    
    // Créer les cellules de la grille
    // Première ligne (en-têtes de colonnes)
    const headerRow = document.createElement('div');
    headerRow.className = 'grid-cell grid-header';
    grid.appendChild(headerRow);
    
    for (let i = 1; i <= 6; i++) {
        const columnHeader = document.createElement('div');
        columnHeader.className = 'grid-cell grid-header';
        columnHeader.textContent = i;
        grid.appendChild(columnHeader);
    }
    
    // Lignes de A à E
    const rows = ['A', 'B', 'C', 'D', 'E'];
    rows.forEach(row => {
        // En-tête de ligne
        const rowHeader = document.createElement('div');
        rowHeader.className = 'grid-cell grid-header';
        rowHeader.textContent = row;
        grid.appendChild(rowHeader);
        
        // Cellules de la ligne
        for (let col = 1; col <= 6; col++) {
            const cell = document.createElement('div');
            cell.className = 'grid-cell';
            cell.id = `cell-${row}${col}`;
            
            // Ajouter le point cible et l'aiguille dans des cellules spécifiques
            if (row === 'B' && col === 5) {
                const target = document.createElement('div');
                target.style.width = '15px';
                target.style.height = '15px';
                target.style.border = '2px dashed #e74c3c';
                target.style.borderRadius = '50%';
                cell.appendChild(target);
            }
            
            if (row === 'C' && col === 3) {
                const needleContainer = document.createElement('div');
                needleContainer.style.position = 'relative';
                
                const needlePoint = document.createElement('div');
                needlePoint.style.width = '10px';
                needlePoint.style.height = '10px';
                needlePoint.style.backgroundColor = '#4caf50';
                needlePoint.style.borderRadius = '50%';
                
                const needlePath = document.createElement('div');
                needlePath.style.width = '2px';
                needlePath.style.height = '30px';
                needlePath.style.backgroundColor = '#4caf50';
                needlePath.style.position = 'absolute';
                needlePath.style.bottom = '0';
                needlePath.style.left = '4px';
                needlePath.style.transform = 'rotate(45deg)';
                needlePath.style.transformOrigin = 'bottom';
                
                needleContainer.appendChild(needlePoint);
                needleContainer.appendChild(needlePath);
                cell.appendChild(needleContainer);
            }
            
            grid.appendChild(cell);
        }
    });
    
    // Affichage des coordonnées
    const coordinates = document.createElement('div');
    coordinates.className = 'coordinates';
    
    // Coordonnée X
    const coordX = document.createElement('div');
    coordX.className = 'coordinate';
    coordX.id = 'coordX';
    coordX.innerHTML = 'X: <span class="positive">+28</span>';
    
    // Coordonnée Y
    const coordY = document.createElement('div');
    coordY.className = 'coordinate';
    coordY.id = 'coordY';
    coordY.innerHTML = 'Y: <span class="negative">-16</span>';
    
    // Coordonnée Z
    const coordZ = document.createElement('div');
    coordZ.className = 'coordinate';
    coordZ.id = 'coordZ';
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
    resetButton.id = 'resetButton';
    
    // Bouton d'arrêt d'urgence
    const emergencyButton = document.createElement('button');
    emergencyButton.className = 'action-btn emergency-btn';
    emergencyButton.textContent = 'Arrêt d\'urgence';
    emergencyButton.id = 'emergencyButton';
    
    // Ajouter les boutons au conteneur
    actionButtons.appendChild(resetButton);
    actionButtons.appendChild(emergencyButton);
    
    // Assembler tous les éléments dans le panneau
    panel.appendChild(title);
    panel.appendChild(grid);
    panel.appendChild(coordinates);
    panel.appendChild(actionButtons);
    
    return panel;
}

// Configuration des écouteurs d'événements pour la page Mode 1
function setupMode1EventListeners() {
    console.log('Configuration des écouteurs d\'événements pour la page Mode 1');
    
    // Gestionnaire pour le curseur de vitesse
    const speedSlider = document.getElementById('speedSlider');
    if (speedSlider) {
        speedSlider.addEventListener('input', function() {
            const value = this.value;
            
            // Mettre à jour l'affichage du curseur
            this.style.background = `linear-gradient(to right, #4caf50 ${value}%, #e0e0e0 ${value}%)`;
            
            console.log('Vitesse ajustée à:', value);
        });
    }
    
    // Gestionnaires pour les boutons
    const resetButton = document.getElementById('resetButton');
    if (resetButton) {
        resetButton.addEventListener('click', function() {
            console.log('Réinitialisation du système');
            
            // Réinitialiser les coordonnées
            updateNeedleCoordinates(0, 0, 0);
            
            // Réinitialiser la position visuelle de l'aiguille (à implémenter)
        });
    }
    
    const emergencyButton = document.getElementById('emergencyButton');
    if (emergencyButton) {
        emergencyButton.addEventListener('click', function() {
            console.log('ARRÊT D\'URGENCE ACTIVÉ');
            alert('ARRÊT D\'URGENCE : Tous les mouvements ont été arrêtés.');
        });
    }
    
    // Simulation de l'interactivité des joysticks (à développer davantage)
    setupJoystickInteractions();
}

// Fonction pour configurer l'interactivité des joysticks
function setupJoystickInteractions() {
    // Le joystick 1 (XY)
    const joystick1 = document.getElementById('joystick1');
    if (joystick1) {
        // Flèches du joystick 1
        const arrows = joystick1.querySelectorAll('.arrow');
        arrows.forEach(arrow => {
            arrow.addEventListener('mousedown', function() {
                this.style.opacity = '0.5';
                
                // Identifier la direction et mettre à jour les coordonnées
                const direction = this.className.split('arrow-')[1];
                updateJoystickMovement(direction);
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
    const joystick2 = document.getElementById('joystick2');
    if (joystick2) {
        // Flèches du joystick 2
        const arrows = joystick2.querySelectorAll('.arrow');
        arrows.forEach(arrow => {
            arrow.addEventListener('mousedown', function() {
                this.style.opacity = '0.5';
                
                // Identifier la direction et mettre à jour les coordonnées
                const direction = this.className.split('arrow-')[1];
                updateJoystickZMovement(direction);
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

// Fonction pour mettre à jour le mouvement XY en fonction du joystick 1
function updateJoystickMovement(direction) {
    console.log('Mouvement du joystick 1:', direction);
    
    // Récupérer les coordonnées actuelles
    const xElement = document.querySelector('#coordX span');
    const yElement = document.querySelector('#coordY span');
    
    // Extraire les valeurs numériques (retirer le signe +/-)
    let x = parseInt(xElement.textContent);
    let y = parseInt(yElement.textContent);
    
    // Mise à jour en fonction de la direction
    const speed = parseInt(document.getElementById('speedSlider').value) / 10;
    
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
    updateNeedleCoordinates(x, y, null);
}

// Fonction pour mettre à jour le mouvement Z en fonction du joystick 2
function updateJoystickZMovement(direction) {
    console.log('Mouvement du joystick 2:', direction);
    
    // Récupérer la coordonnée Z actuelle
    const zElement = document.querySelector('#coordZ span');
    
    // Extraire la valeur numérique (retirer le signe +/-)
    let z = parseInt(zElement.textContent);
    
    // Mise à jour en fonction de la direction
    const speed = parseInt(document.getElementById('speedSlider').value) / 10;
    
    switch(direction) {
        case 'up':
            z += speed;
            break;
        case 'down':
            z -= speed;
            break;
    }
    
    // Mettre à jour la coordonnée Z affichée
    updateNeedleCoordinates(null, null, z);
}

// Fonction pour mettre à jour les coordonnées de l'aiguille
function updateNeedleCoordinates(x, y, z) {
    // Mettre à jour X si fourni
    if (x !== null) {
        const xElement = document.querySelector('#coordX span');
        xElement.textContent = (x >= 0 ? '+' : '') + x;
        xElement.className = x >= 0 ? 'positive' : 'negative';
    }
    
    // Mettre à jour Y si fourni
    if (y !== null) {
        const yElement = document.querySelector('#coordY span');
        yElement.textContent = (y >= 0 ? '+' : '') + y;
        yElement.className = y >= 0 ? 'positive' : 'negative';
    }
    
    // Mettre à jour Z si fourni
    if (z !== null) {
        const zElement = document.querySelector('#coordZ span');
        zElement.textContent = (z >= 0 ? '+' : '') + z;
        zElement.className = z >= 0 ? 'positive' : 'negative';
    }
    
    // À faire: Mettre à jour la position visuelle de l'aiguille dans la grille
    // updateNeedleVisualization(x, y, z);
}

// Fonction pour mettre à jour la visualisation de l'aiguille (à implémenter)
function updateNeedleVisualization(x, y, z) {
    // Cette fonction déplacerait visuellement l'aiguille dans la grille
    // en fonction des coordonnées
    console.log('Mise à jour de la visualisation de l\'aiguille:', { x, y, z });
    
    // Implémentation à développer selon les besoins spécifiques
}

// Fonction pour nettoyer la page lors du changement de page
function cleanupMode1Page() {
    console.log('Nettoyage de la page Mode 1');
    
    // Supprimer les écouteurs d'événements pour éviter les fuites de mémoire
    
    // Réinitialiser les joysticks
    const joystick1 = document.getElementById('joystick1');
    if (joystick1) {
        const arrows = joystick1.querySelectorAll('.arrow');
        arrows.forEach(arrow => {
            arrow.removeEventListener('mousedown', null);
            arrow.removeEventListener('mouseup', null);
            arrow.removeEventListener('mouseleave', null);
        });
    }
    
    const joystick2 = document.getElementById('joystick2');
    if (joystick2) {
        const arrows = joystick2.querySelectorAll('.arrow');
        arrows.forEach(arrow => {
            arrow.removeEventListener('mousedown', null);
            arrow.removeEventListener('mouseup', null);
            arrow.removeEventListener('mouseleave', null);
        });
    }
    
    // Réinitialiser le curseur de vitesse
    const speedSlider = document.getElementById('speedSlider');
    if (speedSlider) {
        speedSlider.removeEventListener('input', null);
    }
    
    // Réinitialiser les boutons d'action
    const resetButton = document.getElementById('resetButton');
    if (resetButton) {
        resetButton.removeEventListener('click', null);
    }
    
    const emergencyButton = document.getElementById('emergencyButton');
    if (emergencyButton) {
        emergencyButton.removeEventListener('click', null);
    }
}

// Exportation des fonctions pour utilisation dans le module principal
export {
    initMode1Page,
    updateNeedleCoordinates,
    cleanupMode1Page
};