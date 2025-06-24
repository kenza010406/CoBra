// page3.js - Mode 1 (VERSION COMPLÈTE FINALE)
// Gestion de la page Mode 1 du système CoBra avec système de coordonnées correct

// Système de coordonnées de la grille (D4 = origine 0,0)
const GRID_COORDINATES = {
    'A1': { x: -15, y: 15 }, 'B1': { x: -10, y: 15 }, 'C1': { x: -5, y: 15 }, 'D1': { x: 0, y: 15 }, 'E1': { x: 5, y: 15 }, 'F1': { x: 10, y: 15 }, 'G1': { x: 15, y: 15 },
    'A2': { x: -15, y: 10 }, 'B2': { x: -10, y: 10 }, 'C2': { x: -5, y: 10 }, 'D2': { x: 0, y: 10 }, 'E2': { x: 5, y: 10 }, 'F2': { x: 10, y: 10 }, 'G2': { x: 15, y: 10 },
    'A3': { x: -15, y: 5 },  'B3': { x: -10, y: 5 },  'C3': { x: -5, y: 5 },  'D3': { x: 0, y: 5 },  'E3': { x: 5, y: 5 },  'F3': { x: 10, y: 5 },  'G3': { x: 15, y: 5 },
    'A4': { x: -15, y: 0 },  'B4': { x: -10, y: 0 },  'C4': { x: -5, y: 0 },  'D4': { x: 0, y: 0 },   'E4': { x: 5, y: 0 },   'F4': { x: 10, y: 0 },  'G4': { x: 15, y: 0 },
    'A5': { x: -15, y: -5 }, 'B5': { x: -10, y: -5 }, 'C5': { x: -5, y: -5 }, 'D5': { x: 0, y: -5 },  'E5': { x: 5, y: -5 },  'F5': { x: 10, y: -5 }, 'G5': { x: 15, y: -5 },
    'A6': { x: -15, y: -10 },'B6': { x: -10, y: -10 },'C6': { x: -5, y: -10 },'D6': { x: 0, y: -10 }, 'E6': { x: 5, y: -10 }, 'F6': { x: 10, y: -10 },'G6': { x: 15, y: -10 },
    'A7': { x: -15, y: -15 },'B7': { x: -10, y: -15 },'C7': { x: -5, y: -15 },'D7': { x: 0, y: -15 }, 'E7': { x: 5, y: -15 }, 'F7': { x: 10, y: -15 },'G7': { x: 15, y: -15 }
};

// Position actuelle de l'aiguille - MODIFIÉE : position de départ Z=0
let currentPosition = { x: 0, y: 0, z: 0 };

// Fonction d'initialisation de la page Mode 1
function initMode1Page() {
    console.log('Initialisation de la page Mode 1 (version complète finale)');
    
    // Initialiser la position par défaut à D4 (0,0,0) - MODIFIÉE : Z=0
    currentPosition = { x: 0, y: 0, z: 0 };
    
    // Enregistrer directement les gestionnaires d'événements sur les éléments existants
    setupMode1EventListeners();
    
    // Initialiser les curseurs existants
    initializeSliders();
    
    // Initialiser l'affichage des coordonnées - MODIFIÉE : Z=0
    updateNeedleCoordinates(0, 0, 0);
    
    // Positionner l'aiguille sur D4 au démarrage
    positionNeedleOnGrid('D4');
    
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
            
            // Réinitialiser à la position D4 (origine) - MODIFIÉE : Z=0
            currentPosition = { x: 0, y: 0, z: 0 };
            updateNeedleCoordinates(0, 0, 0);
            positionNeedleOnGrid('D4');
            
            // Réinitialiser les valeurs de mode direct
            resetDirectPositionInputs();
            
            alert('Système réinitialisé à la position D4 (origine).');
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
    
    // Configuration du clic sur les cellules de la grille
    setupGridClickHandlers();
}

// Fonction pour configurer les clics sur la grille
function setupGridClickHandlers() {
    const mode1Page = document.getElementById('mode1');
    const gridCells = mode1Page.querySelectorAll('.grid-cell[data-pos]');
    
    gridCells.forEach(cell => {
        cell.addEventListener('click', function() {
            const position = this.dataset.pos;
            const x = parseInt(this.dataset.x);
            const y = parseInt(this.dataset.y);
            
            // Mettre à jour la position actuelle
            currentPosition.x = x;
            currentPosition.y = y;
            
            // Mettre à jour l'affichage
            updateNeedleCoordinates(x, y, currentPosition.z);
            positionNeedleOnGrid(position);
            
            // Mettre à jour les inputs du mode direct
            document.getElementById('directX').value = x;
            document.getElementById('directY').value = y;
            
            console.log(`Position sélectionnée: ${position} (${x}, ${y})`);
        });
    });
}

// Fonction pour positionner l'aiguille sur la grille
function positionNeedleOnGrid(gridPosition) {
    // Supprimer l'ancienne position de l'aiguille
    const oldNeedles = document.querySelectorAll('#mode1 .needle-indicator');
    oldNeedles.forEach(needle => {
        if (needle.parentElement.id !== 'centerPosition') {
            needle.remove();
        }
    });
    
    // Trouver la cellule correspondante
    const mode1Page = document.getElementById('mode1');
    const targetCell = mode1Page.querySelector(`[data-pos="${gridPosition}"]`);
    
    if (targetCell) {
        // Si ce n'est pas la position D4, créer un nouvel indicateur
        if (gridPosition !== 'D4') {
            // Masquer l'indicateur de D4 si présent
            const d4Cell = document.getElementById('centerPosition');
            if (d4Cell) {
                const d4Needle = d4Cell.querySelector('.needle-indicator');
                if (d4Needle) {
                    d4Needle.style.display = 'none';
                }
            }
            
            // Créer le nouvel indicateur d'aiguille
            const needleIndicator = document.createElement('div');
            needleIndicator.className = 'needle-indicator';
            needleIndicator.innerHTML = '<div class="needle-circle"></div>';
            
            targetCell.appendChild(needleIndicator);
        } else {
            // Réafficher l'indicateur de D4
            const d4Cell = document.getElementById('centerPosition');
            if (d4Cell) {
                const d4Needle = d4Cell.querySelector('.needle-indicator');
                if (d4Needle) {
                    d4Needle.style.display = 'block';
                }
            }
        }
        
        console.log(`Aiguille positionnée sur ${gridPosition}`);
    }
}

// Fonction pour trouver la position grille initiale
function findClosestGridPosition(x, y) {
    let closestPosition = 'D4';
    let minDistance = Infinity;
    
    for (const [position, coords] of Object.entries(GRID_COORDINATES)) {
        const distance = Math.sqrt(Math.pow(x - coords.x, 2) + Math.pow(y - coords.y, 2));
        if (distance < minDistance) {
            minDistance = distance;
            closestPosition = position;
        }
    }
    
    return closestPosition;
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
        
        // Configuration du joystick 1 (XY)
        const arrows1 = joystick1.querySelectorAll('.arrow');
        arrows1.forEach(arrow => {
            arrow.addEventListener('mousedown', function(e) {
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
    
    // Gestionnaires pour les boutons +/- - MODIFIÉE : pas de 5
    document.querySelectorAll('#mode1 .coord-btn').forEach(btn => {
        btn.addEventListener('click', function(e) {
            e.preventDefault();
            
            const axis = this.dataset.axis;
            const isPlus = this.classList.contains('plus');
            const inputId = `direct${axis.toUpperCase()}`;
            const input = document.getElementById(inputId);
            
            if (input) {
                let value = parseInt(input.value) || 0;
                const step = 5; // Pas d'incrémentation de 5 - CONFIRMÉ
                
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
                // Mettre à jour la position actuelle
                currentPosition = { x, y, z };
                
                // Mettre à jour les coordonnées affichées
                updateNeedleCoordinates(x, y, z);
                
                // Trouver et positionner sur la grille
                const closestPosition = findClosestGridPosition(x, y);
                positionNeedleOnGrid(closestPosition);
                
                // Réinitialiser le bouton
                this.style.transform = 'scale(1)';
                this.textContent = 'Exécuter';
                this.disabled = false;
                
                alert(`Position atteinte: ${closestPosition} (X:${x}, Y:${y}, Z:${z})`);
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

// Fonction pour réinitialiser les inputs de position directe - MODIFIÉE : Z=0
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

// Fonction pour mettre à jour le mouvement XY en fonction du joystick 1 - MODIFIÉE : pas de 5
function updateJoystickMovement(direction) {
    console.log('Mouvement du joystick 1:', direction);
    
    // MODIFIÉE : Pas fixe de 5 pour le joystick
    const step = 5;
    
    // Mise à jour en fonction de la direction avec pas de 5
    switch(direction) {
        case 'up':
            currentPosition.y += step;
            break;
        case 'down':
            currentPosition.y -= step;
            break;
        case 'left':
            currentPosition.x -= step;
            break;
        case 'right':
            currentPosition.x += step;
            break;
    }
    
    // Limiter les valeurs
    currentPosition.x = Math.max(-15, Math.min(15, currentPosition.x));
    currentPosition.y = Math.max(-15, Math.min(15, currentPosition.y));
    
    // Mettre à jour les coordonnées affichées
    updateNeedleCoordinates(currentPosition.x, currentPosition.y, null);
    
    // Trouver et positionner sur la grille
    const closestPosition = findClosestGridPosition(currentPosition.x, currentPosition.y);
    positionNeedleOnGrid(closestPosition);
    
    // Mettre à jour les inputs du mode direct
    document.getElementById('directX').value = currentPosition.x;
    document.getElementById('directY').value = currentPosition.y;
    
    console.log('Nouvelles coordonnées XY:', currentPosition.x, currentPosition.y);
}

// Fonction pour mettre à jour le mouvement Z en fonction du joystick 2 - MODIFIÉE : pas de 5
function updateJoystickZMovement(direction) {
    console.log('Mouvement du joystick 2:', direction);
    
    // MODIFIÉE : Pas fixe de 5 pour le joystick Z
    const step = 5;
    
    // Mise à jour en fonction de la direction avec pas de 5
    switch(direction) {
        case 'up':
            currentPosition.z += step;
            break;
        case 'down':
            currentPosition.z -= step;
            break;
    }
    
    // Limiter les valeurs
    currentPosition.z = Math.max(-20, Math.min(20, currentPosition.z));
    
    // Mettre à jour la coordonnée Z affichée
    updateNeedleCoordinates(null, null, currentPosition.z);
    
    // Mettre à jour l'input du mode direct
    document.getElementById('directZ').value = currentPosition.z;
    
    console.log('Nouvelle coordonnée Z:', currentPosition.z);
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
            if (x > 0) xElement.className = 'positive';
            else if (x < 0) xElement.className = 'negative';
            else xElement.className = 'neutral';
        }
        currentPosition.x = x;
    }
    
    // Mettre à jour Y si fourni
    if (y !== null) {
        const yElement = mode1Page.querySelector('.coordinate:nth-child(2) span');
        if (yElement) {
            yElement.textContent = (y >= 0 ? '+' : '') + y;
            if (y > 0) yElement.className = 'positive';
            else if (y < 0) yElement.className = 'negative';
            else yElement.className = 'neutral';
        }
        currentPosition.y = y;
    }
    
    // Mettre à jour Z si fourni
    if (z !== null) {
        const zElement = mode1Page.querySelector('.coordinate:nth-child(3) span');
        if (zElement) {
            zElement.textContent = (z >= 0 ? '+' : '') + z;
            if (z > 0) zElement.className = 'positive';
            else if (z < 0) zElement.className = 'negative';
            else zElement.className = 'neutral';
        }
        currentPosition.z = z;
    }
    
    console.log('Coordonnées mises à jour - X:', currentPosition.x, 'Y:', currentPosition.y, 'Z:', currentPosition.z);
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
        
        // Nettoyer les cellules de grille
        const gridCells = mode1Page.querySelectorAll('.grid-cell[data-pos]');
        gridCells.forEach(cell => {
            const newCell = cell.cloneNode(true);
            cell.parentNode.replaceChild(newCell, cell);
        });
    }
}

// Exportation des fonctions pour utilisation dans le module principal
export {
    initMode1Page,
    updateNeedleCoordinates,
    cleanupMode1Page
};
