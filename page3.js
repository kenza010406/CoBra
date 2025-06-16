// page3.js - Mode 1 (VERSION CORRIGÉE AVEC MODE DIRECT)
// Gestion de la page Mode 1 du système CoBra (contrôle grille de coordonnées + mode direct)

// Fonction d'initialisation de la page Mode 1
function initMode1Page() {
    console.log('Initialisation de la page Mode 1 (version corrigée avec mode direct)');
    
    // Ne pas créer de nouveaux éléments, utiliser ceux qui existent déjà dans le HTML
    // Enregistrer directement les gestionnaires d'événements sur les éléments existants
    setupMode1EventListeners();
    
    // Initialiser les curseurs existants
    initializeSliders();
    
    return document.getElementById('mode1');
}

// Fonction pour initialiser les curseurs
function initializeSliders() {
    const speedSlider = document.getElementById('speedSlider');
    if (speedSlider) {
        // Initialiser l'apparence du curseur
        const value = speedSlider.value;
        speedSlider.style.background = `linear-gradient(to right, #4caf50 ${value}%, #e0e0e0 ${value}%)`;
    }
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
            
            // Réinitialiser les valeurs de mode direct
            resetDirectPositionInputs();
            
            // Afficher une notification
            alert('Système réinitialisé avec succès.');
        });
    }
    
    const emergencyButton = document.getElementById('emergencyButton');
    if (emergencyButton) {
        emergencyButton.addEventListener('click', function() {
            console.log('ARRÊT D\'URGENCE ACTIVÉ');
            alert('ARRÊT D\'URGENCE : Tous les mouvements ont été arrêtés.');
        });
    }
    
    // Configuration de l'interactivité des joysticks existants
    setupJoystickInteractions();
    
    // Configuration des contrôles de position directe
    setupDirectPositionControls();
}

// Fonction pour configurer l'interactivité des joysticks
function setupJoystickInteractions() {
    console.log('Configuration des joysticks Mode 1');
    
    // Trouver tous les joysticks dans la page mode1
    const mode1Page = document.getElementById('mode1');
    if (!mode1Page) {
        console.error('Page Mode 1 non trouvée');
        return;
    }
    
    // Sélectionner les joysticks dans la page Mode 1
    const joysticks = mode1Page.querySelectorAll('.joystick');
    
    if (joysticks.length >= 2) {
        const joystick1 = joysticks[0]; // Premier joystick (XY)
        const joystick2 = joysticks[1]; // Deuxième joystick (Z)
        
        console.log('Joysticks trouvés:', joystick1, joystick2);
        
        // Configuration du joystick 1 (XY)
        const arrows1 = joystick1.querySelectorAll('.arrow');
        arrows1.forEach(arrow => {
            arrow.addEventListener('mousedown', function(e) {
                e.preventDefault();
                this.style.opacity = '0.5';
                
                // Identifier la direction
                let direction = '';
                if (this.classList.contains('arrow-up')) direction = 'up';
                else if (this.classList.contains('arrow-down')) direction = 'down';
                else if (this.classList.contains('arrow-left')) direction = 'left';
                else if (this.classList.contains('arrow-right')) direction = 'right';
                
                if (direction) {
                    updateJoystickMovement(direction);
                }
            });
            
            arrow.addEventListener('mouseup', function() {
                this.style.opacity = '1';
            });
            
            arrow.addEventListener('mouseleave', function() {
                this.style.opacity = '1';
            });
            
            // Support tactile
            arrow.addEventListener('touchstart', function(e) {
                e.preventDefault();
                this.style.opacity = '0.5';
                
                let direction = '';
                if (this.classList.contains('arrow-up')) direction = 'up';
                else if (this.classList.contains('arrow-down')) direction = 'down';
                else if (this.classList.contains('arrow-left')) direction = 'left';
                else if (this.classList.contains('arrow-right')) direction = 'right';
                
                if (direction) {
                    updateJoystickMovement(direction);
                }
            });
            
            arrow.addEventListener('touchend', function(e) {
                e.preventDefault();
                this.style.opacity = '1';
            });
        });
        
        // Configuration du joystick 2 (Z)
        const arrows2 = joystick2.querySelectorAll('.arrow');
        arrows2.forEach(arrow => {
            arrow.addEventListener('mousedown', function(e) {
                e.preventDefault();
                this.style.opacity = '0.5';
                
                // Identifier la direction
                let direction = '';
                if (this.classList.contains('arrow-up')) direction = 'up';
                else if (this.classList.contains('arrow-down')) direction = 'down';
                
                if (direction) {
                    updateJoystickZMovement(direction);
                }
            });
            
            arrow.addEventListener('mouseup', function() {
                this.style.opacity = '1';
            });
            
            arrow.addEventListener('mouseleave', function() {
                this.style.opacity = '1';
            });
            
            // Support tactile
            arrow.addEventListener('touchstart', function(e) {
                e.preventDefault();
                this.style.opacity = '0.5';
                
                let direction = '';
                if (this.classList.contains('arrow-up')) direction = 'up';
                else if (this.classList.contains('arrow-down')) direction = 'down';
                
                if (direction) {
                    updateJoystickZMovement(direction);
                }
            });
            
            arrow.addEventListener('touchend', function(e) {
                e.preventDefault();
                this.style.opacity = '1';
            });
        });
        
        console.log('Joysticks Mode 1 configurés avec succès');
    } else {
        console.error('Joysticks non trouvés dans Mode 1. Nombre trouvé:', joysticks.length);
    }
}

// Nouvelle fonction pour configurer les contrôles de position directe
function setupDirectPositionControls() {
    console.log('Configuration des contrôles de position directe');
    
    // Gestionnaires pour les boutons +/-
    document.querySelectorAll('#mode1 .coord-btn').forEach(btn => {
        btn.addEventListener('click', function(e) {
            e.preventDefault();
            
            const axis = this.dataset.axis;
            const isPlus = this.classList.contains('plus');
            const inputId = `direct${axis.toUpperCase()}`;
            const input = document.getElementById(inputId);
            
            if (input) {
                let value = parseInt(input.value) || 0;
                const step = 5; // Pas d'incrémentation
                
                if (isPlus) {
                    value = Math.min(parseInt(input.max), value + step);
                } else {
                    value = Math.max(parseInt(input.min), value - step);
                }
                
                input.value = value;
                
                // Animation du bouton
                this.style.transform = 'scale(0.95)';
                setTimeout(() => {
                    this.style.transform = 'scale(1)';
                }, 100);
                
                console.log(`Valeur ${axis.toUpperCase()} mise à jour:`, value);
            }
        });
    });
    
    // Gestionnaire pour le bouton Exécuter
    const executeBtn = document.getElementById('executeDirectPosition');
    if (executeBtn) {
        executeBtn.addEventListener('click', function(e) {
            e.preventDefault();
            
            const x = parseInt(document.getElementById('directX').value) || 0;
            const y = parseInt(document.getElementById('directY').value) || 0;
            const z = parseInt(document.getElementById('directZ').value) || 0;
            
            console.log('Exécution position directe:', { x, y, z });
            
            // Animation du bouton
            this.style.transform = 'scale(0.95)';
            this.textContent = 'Exécution...';
            this.disabled = true;
            
            // Simuler le déplacement
            setTimeout(() => {
                // Mettre à jour les coordonnées affichées
                updateNeedleCoordinates(x, y, z);
                
                // Déplacer visuellement l'aiguille dans la grille
                updateNeedleVisualPosition(x, y);
                
                // Réinitialiser le bouton
                this.style.transform = 'scale(1)';
                this.textContent = 'Exécuter';
                this.disabled = false;
                
                alert(`Position atteinte: X:${x}, Y:${y}, Z:${z}`);
            }, 1500);
        });
    }
    
    // Validation en temps réel des inputs
    ['directX', 'directY', 'directZ'].forEach(id => {
        const input = document.getElementById(id);
        if (input) {
            input.addEventListener('input', function() {
                let value = parseInt(this.value);
                const min = parseInt(this.min);
                const max = parseInt(this.max);
                
                // Validation des limites
                if (isNaN(value)) {
                    this.value = min;
                    return;
                }
                
                if (value < min) {
                    this.value = min;
                    value = min;
                }
                if (value > max) {
                    this.value = max;
                    value = max;
                }
                
                // Changement de couleur selon validation
                if (value >= min && value <= max) {
                    this.style.backgroundColor = '#f1f8e9';
                    this.style.borderColor = '#4caf50';
                } else {
                    this.style.backgroundColor = '#ffebee';
                    this.style.borderColor = '#e74c3c';
                }
                
                console.log(`Input ${id} validé:`, value);
            });
            
            // Validation au focus lost
            input.addEventListener('blur', function() {
                const value = parseInt(this.value);
                const min = parseInt(this.min);
                const max = parseInt(this.max);
                
                if (isNaN(value) || value < min || value > max) {
                    this.value = min;
                    this.style.backgroundColor = '';
                    this.style.borderColor = '';
                }
            });
        }
    });
}

// Fonction pour réinitialiser les inputs de position directe
function resetDirectPositionInputs() {
    document.getElementById('directX').value = 0;
    document.getElementById('directY').value = 0;
    document.getElementById('directZ').value = 0;
    
    // Réinitialiser les styles
    ['directX', 'directY', 'directZ'].forEach(id => {
        const input = document.getElementById(id);
        if (input) {
            input.style.backgroundColor = '';
            input.style.borderColor = '';
        }
    });
}

// Fonction pour mettre à jour la position visuelle de l'aiguille dans la grille
function updateNeedleVisualPosition(x, y) {
    // Supprimer l'ancienne position de l'aiguille
    const oldNeedle = document.querySelector('#mode1 .needle-indicator');
    if (oldNeedle) {
        oldNeedle.remove();
    }
    
    // Calculer la position dans la grille (7x7)
    // Conversion des coordonnées X,Y en position grille
    let gridCol = Math.round((x + 100) / 200 * 6) + 1; // Colonnes A-G (1-7)
    let gridRow = Math.round((y + 100) / 200 * 6) + 1; // Lignes 1-7
    
    // Limiter aux bornes de la grille
    gridCol = Math.max(1, Math.min(7, gridCol));
    gridRow = Math.max(1, Math.min(7, gridRow));
    
    // Trouver la cellule correspondante dans la grille
    const mode1Page = document.getElementById('mode1');
    const gridCells = mode1Page.querySelectorAll('.grid-cell:not(.grid-header)');
    
    // Calculer l'index de la cellule (grille 8x8 avec headers)
    const cellIndex = (gridRow - 1) * 8 + gridCol;
    
    if (gridCells[cellIndex]) {
        // Créer le nouvel indicateur d'aiguille
        const needleIndicator = document.createElement('div');
        needleIndicator.className = 'needle-indicator';
        needleIndicator.innerHTML = '<div class="needle-circle"></div>';
        
        gridCells[cellIndex].appendChild(needleIndicator);
        
        console.log(`Aiguille déplacée vers la position grille: Ligne ${gridRow}, Colonne ${String.fromCharCode(64 + gridCol)}`);
    }
}

// Fonction pour mettre à jour le mouvement XY en fonction du joystick 1
function updateJoystickMovement(direction) {
    console.log('Mouvement du joystick 1:', direction);
    
    // Récupérer les coordonnées actuelles (chercher dans la page mode1 spécifiquement)
    const mode1Page = document.getElementById('mode1');
    const xElement = mode1Page.querySelector('.coordinate:nth-child(1) span');
    const yElement = mode1Page.querySelector('.coordinate:nth-child(2) span');
    
    if (!xElement || !yElement) {
        console.error('Éléments de coordonnées non trouvés');
        return;
    }
    
    // Extraire les valeurs numériques (retirer le signe +/-)
    let x = parseInt(xElement.textContent.replace(/[^-\d]/g, '')) || 0;
    let y = parseInt(yElement.textContent.replace(/[^-\d]/g, '')) || 0;
    
    // Récupérer la vitesse
    const speedSlider = document.getElementById('speedSlider');
    const speed = speedSlider ? Math.max(1, parseInt(speedSlider.value) / 10) : 5;
    
    // Mise à jour en fonction de la direction
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
    
    // Limiter les valeurs (optionnel)
    x = Math.max(-100, Math.min(100, x));
    y = Math.max(-100, Math.min(100, y));
    
    // Mettre à jour les coordonnées affichées
    updateNeedleCoordinates(x, y, null);
    
    // Mettre à jour la position visuelle
    updateNeedleVisualPosition(x, y);
    
    console.log('Nouvelles coordonnées XY:', x, y);
}

// Fonction pour mettre à jour le mouvement Z en fonction du joystick 2
function updateJoystickZMovement(direction) {
    console.log('Mouvement du joystick 2:', direction);
    
    // Récupérer la coordonnée Z actuelle
    const mode1Page = document.getElementById('mode1');
    const zElement = mode1Page.querySelector('.coordinate:nth-child(3) span');
    
    if (!zElement) {
        console.error('Élément de coordonnée Z non trouvé');
        return;
    }
    
    // Extraire la valeur numérique (retirer le signe +/-)
    let z = parseInt(zElement.textContent.replace(/[^-\d]/g, '')) || 0;
    
    // Récupérer la vitesse
    const speedSlider = document.getElementById('speedSlider');
    const speed = speedSlider ? Math.max(1, parseInt(speedSlider.value) / 10) : 5;
    
    // Mise à jour en fonction de la direction
    switch(direction) {
        case 'up':
            z += speed;
            break;
        case 'down':
            z -= speed;
            break;
    }
    
    // Limiter les valeurs (optionnel)
    z = Math.max(-50, Math.min(50, z));
    
    // Mettre à jour la coordonnée Z affichée
    updateNeedleCoordinates(null, null, z);
    
    console.log('Nouvelle coordonnée Z:', z);
}

// Fonction pour mettre à jour les coordonnées de l'aiguille
function updateNeedleCoordinates(x, y, z) {
    const mode1Page = document.getElementById('mode1');
    if (!mode1Page) return;
    
    // Mettre à jour X si fourni
    if (x !== null) {
        const xElement = mode1Page.querySelector('.coordinate:nth-child(1) span');
        if (xElement) {
            xElement.textContent = (x >= 0 ? '+' : '') + x;
            xElement.className = x >= 0 ? 'positive' : 'negative';
        }
    }
    
    // Mettre à jour Y si fourni
    if (y !== null) {
        const yElement = mode1Page.querySelector('.coordinate:nth-child(2) span');
        if (yElement) {
            yElement.textContent = (y >= 0 ? '+' : '') + y;
            yElement.className = y >= 0 ? 'positive' : 'negative';
        }
    }
    
    // Mettre à jour Z si fourni
    if (z !== null) {
        const zElement = mode1Page.querySelector('.coordinate:nth-child(3) span');
        if (zElement) {
            zElement.textContent = (z >= 0 ? '+' : '') + z;
            zElement.className = z >= 0 ? 'positive' : 'negative';
        }
    }
    
    console.log('Coordonnées mises à jour - X:', x, 'Y:', y, 'Z:', z);
}

// Fonction pour nettoyer la page lors du changement de page
function cleanupMode1Page() {
    console.log('Nettoyage de la page Mode 1');
    
    // Supprimer les écouteurs d'événements pour éviter les fuites de mémoire
    const mode1Page = document.getElementById('mode1');
    if (mode1Page) {
        // Nettoyer les joysticks
        const joysticks = mode1Page.querySelectorAll('.joystick');
        joysticks.forEach(joystick => {
            const arrows = joystick.querySelectorAll('.arrow');
            arrows.forEach(arrow => {
                // Cloner l'élément pour supprimer tous les événements
                const newArrow = arrow.cloneNode(true);
                arrow.parentNode.replaceChild(newArrow, arrow);
            });
        });
        
        // Nettoyer les boutons
        const buttons = mode1Page.querySelectorAll('button');
        buttons.forEach(button => {
            const newButton = button.cloneNode(true);
            button.parentNode.replaceChild(newButton, button);
        });
        
        // Nettoyer les inputs
        const inputs = mode1Page.querySelectorAll('input');
        inputs.forEach(input => {
            const newInput = input.cloneNode(true);
            input.parentNode.replaceChild(newInput, input);
        });
    }
}

// Exportation des fonctions pour utilisation dans le module principal
export {
    initMode1Page,
    updateNeedleCoordinates,
    cleanupMode1Page
};
