// page4.js - Mode 2 MODIFIÉ
// Gestion de la page Mode 2 du système CoBra (visualisation prostate avec Sopha)

// Position actuelle de l'aiguille - Mode 2
let currentPositionMode2 = { x: 0, y: 0, z: 0 };

// Fonction d'initialisation de la page Mode 2
function initMode2Page() {
    console.log('Initialisation de la page Mode 2 (version modifiée avec commande directe)');
    
    // Initialiser la position par défaut
    currentPositionMode2 = { x: 0, y: 0, z: 0 };
    
    // Enregistrer directement les gestionnaires d'événements sur les éléments existants
    setupMode2EventListeners();
    
    // Initialiser les curseurs existants
    initializeMode2Sliders();
    
    // Initialiser l'affichage des coordonnées
    updateNeedleCoordinatesMode2(0, 0, 0);
    
    return document.getElementById('mode2');
}

// Fonction pour initialiser les curseurs Mode 2
function initializeMode2Sliders() {
    const speedSlider = document.getElementById('speedSliderMode2');
    if (speedSlider) {
        // Initialiser l'apparence du curseur
        const value = speedSlider.value;
        speedSlider.style.background = `linear-gradient(to right, #4caf50 ${value}%, #e0e0e0 ${value}%)`;
    }
}

// Configuration des écouteurs d'événements pour la page Mode 2
function setupMode2EventListeners() {
    console.log('Configuration des écouteurs d\'événements pour la page Mode 2 (version modifiée)');
    
    // Gestionnaire pour le curseur de vitesse
    const speedSlider = document.getElementById('speedSliderMode2');
    if (speedSlider) {
        speedSlider.addEventListener('input', function() {
            const value = this.value;
            
            // Mettre à jour l'affichage du curseur
            this.style.background = `linear-gradient(to right, #4caf50 ${value}%, #e0e0e0 ${value}%)`;
            
            console.log('Vitesse ajustée à (Mode 2):', value);
        });
    }
    
    // Gestionnaires pour les boutons
    const resetButton = document.getElementById('resetButtonMode2');
    if (resetButton) {
        resetButton.addEventListener('click', function() {
            console.log('Réinitialisation du système (Mode 2)');
            
            // Réinitialiser à la position par défaut
            currentPositionMode2 = { x: 0, y: 0, z: 0 };
            updateNeedleCoordinatesMode2(0, 0, 0);
            
            // Réinitialiser les valeurs de mode direct
            resetDirectPositionInputsMode2();
            
            alert('Système réinitialisé à la position par défaut (Mode 2).');
        });
    }
    
    const emergencyButton = document.getElementById('emergencyButtonMode2');
    if (emergencyButton) {
        emergencyButton.addEventListener('click', function() {
            console.log('ARRÊT D\'URGENCE ACTIVÉ (Mode 2)');
            alert('ARRÊT D\'URGENCE : Tous les mouvements ont été arrêtés (Mode 2).');
        });
    }
    
    // Configuration de l'interactivité des joysticks existants
    setupJoystickInteractionsMode2();
    
    // Configuration des contrôles de position directe
    setupDirectPositionControlsMode2();
}

// Nouvelle fonction pour configurer les contrôles de position directe Mode 2
function setupDirectPositionControlsMode2() {
    console.log('Configuration des contrôles de position directe Mode 2');
    
    // Gestionnaires pour les boutons +/- Mode 2
    document.querySelectorAll('#mode2 .coord-btn').forEach(btn => {
        btn.addEventListener('click', function(e) {
            e.preventDefault();
            
            const axis = this.dataset.axis;
            const isPlus = this.classList.contains('plus');
            const inputId = `directMode2${axis.toUpperCase()}`;
            const input = document.getElementById(inputId);
            
            if (input) {
                let value = parseInt(input.value) || 0;
                const step = 5; // Pas d'incrémentation de 5
                
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
                
                console.log(`Valeur Mode 2 ${axis.toUpperCase()} mise à jour:`, value);
            }
        });
    });
    
    // Gestionnaire pour le bouton Exécuter Mode 2
    const executeBtn = document.getElementById('executeDirectPositionMode2');
    if (executeBtn) {
        executeBtn.addEventListener('click', function(e) {
            e.preventDefault();
            
            const x = parseInt(document.getElementById('directMode2X').value) || 0;
            const y = parseInt(document.getElementById('directMode2Y').value) || 0;
            const z = parseInt(document.getElementById('directMode2Z').value) || 0;
            
            console.log('Exécution position directe Mode 2:', { x, y, z });
            
            // Animation du bouton
            this.style.transform = 'scale(0.95)';
            this.textContent = 'Exécution...';
            this.disabled = true;
            
            // Simuler le déplacement
            setTimeout(() => {
                // Mettre à jour la position actuelle
                currentPositionMode2 = { x, y, z };
                
                // Mettre à jour les coordonnées affichées
                updateNeedleCoordinatesMode2(x, y, z);
                
                // Réinitialiser le bouton
                this.style.transform = 'scale(1)';
                this.textContent = 'Exécuter';
                this.disabled = false;
                
                // Simuler l'intégration avec Sopha
                updateSophaVisualization(x, y, z);
                
                alert(`Position atteinte en Mode 2 (X:${x}, Y:${y}, Z:${z})\nIntégration Sopha mise à jour.`);
            }, 1500);
        });
    }
    
    // Validation en temps réel des inputs Mode 2
    ['directMode2X', 'directMode2Y', 'directMode2Z'].forEach(id => {
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
                
                console.log(`Input Mode 2 ${id} validé:`, value);
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

// Fonction pour réinitialiser les inputs de position directe Mode 2
function resetDirectPositionInputsMode2() {
    document.getElementById('directMode2X').value = 0;
    document.getElementById('directMode2Y').value = 0;
    document.getElementById('directMode2Z').value = 0;
    
    // Réinitialiser les styles
    ['directMode2X', 'directMode2Y', 'directMode2Z'].forEach(id => {
        const input = document.getElementById(id);
        if (input) {
            input.style.backgroundColor = '';
            input.style.borderColor = '';
        }
    });
}

// Fonction pour configurer l'interactivité des joysticks (Mode 2)
function setupJoystickInteractionsMode2() {
    console.log('Configuration des joysticks Mode 2');
    
    // Trouver tous les joysticks dans la page mode2
    const mode2Page = document.getElementById('mode2');
    if (!mode2Page) {
        console.error('Page Mode 2 non trouvée');
        return;
    }
    
    // Sélectionner les joysticks dans la page Mode 2
    const joysticks = mode2Page.querySelectorAll('.joystick');
    
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
                    updateJoystickMovementMode2(direction);
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
                    updateJoystickMovementMode2(direction);
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
                    updateJoystickZMovementMode2(direction);
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
                    updateJoystickZMovementMode2(direction);
                }
            });
            
            arrow.addEventListener('touchend', function(e) {
                e.preventDefault();
                this.style.opacity = '1';
            });
        });
        
        console.log('Joysticks Mode 2 configurés avec succès');
    } else {
        console.error('Joysticks non trouvés dans Mode 2. Nombre trouvé:', joysticks.length);
    }
}

// Fonction pour mettre à jour le mouvement XY en fonction du joystick 1 (Mode 2)
function updateJoystickMovementMode2(direction) {
    console.log('Mouvement du joystick 1 Mode 2:', direction);
    
    // Pas fixe de 5 pour le joystick
    const step = 5;
    
    // Mise à jour en fonction de la direction avec pas de 5
    switch(direction) {
        case 'up':
            currentPositionMode2.y += step;
            break;
        case 'down':
            currentPositionMode2.y -= step;
            break;
        case 'left':
            currentPositionMode2.x -= step;
            break;
        case 'right':
            currentPositionMode2.x += step;
            break;
    }
    
    // Limiter les valeurs (adapté au Mode 2)
    currentPositionMode2.x = Math.max(-100, Math.min(100, currentPositionMode2.x));
    currentPositionMode2.y = Math.max(-100, Math.min(100, currentPositionMode2.y));
    
    // Mettre à jour les coordonnées affichées
    updateNeedleCoordinatesMode2(currentPositionMode2.x, currentPositionMode2.y, null);
    
    // Mettre à jour les inputs du mode direct
    document.getElementById('directMode2X').value = currentPositionMode2.x;
    document.getElementById('directMode2Y').value = currentPositionMode2.y;
    
    // Mettre à jour la visualisation Sopha
    updateSophaVisualization(currentPositionMode2.x, currentPositionMode2.y, currentPositionMode2.z);
    
    console.log('Nouvelles coordonnées XY Mode 2:', currentPositionMode2.x, currentPositionMode2.y);
}

// Fonction pour mettre à jour le mouvement Z en fonction du joystick 2 (Mode 2)
function updateJoystickZMovementMode2(direction) {
    console.log('Mouvement du joystick 2 Mode 2:', direction);
    
    // Pas fixe de 5 pour le joystick Z
    const step = 5;
    
    // Mise à jour en fonction de la direction avec pas de 5
    switch(direction) {
        case 'up':
            currentPositionMode2.z += step;
            break;
        case 'down':
            currentPositionMode2.z -= step;
            break;
    }
    
    // Limiter les valeurs
    currentPositionMode2.z = Math.max(-50, Math.min(50, currentPositionMode2.z));
    
    // Mettre à jour la coordonnée Z affichée
    updateNeedleCoordinatesMode2(null, null, currentPositionMode2.z);
    
    // Mettre à jour l'input du mode direct
    document.getElementById('directMode2Z').value = currentPositionMode2.z;
    
    // Mettre à jour la visualisation Sopha
    updateSophaVisualization(currentPositionMode2.x, currentPositionMode2.y, currentPositionMode2.z);
    
    console.log('Nouvelle coordonnée Z Mode 2:', currentPositionMode2.z);
}

// Fonction pour mettre à jour les coordonnées de l'aiguille (Mode 2)
function updateNeedleCoordinatesMode2(x, y, z) {
    const mode2Page = document.getElementById('mode2');
    if (!mode2Page) return;
    
    // Mettre à jour X si fourni
    if (x !== null) {
        const xElement = mode2Page.querySelector('#coordXMode2 span');
        if (xElement) {
            xElement.textContent = (x >= 0 ? '+' : '') + x;
            if (x > 0) xElement.className = 'positive';
            else if (x < 0) xElement.className = 'negative';
            else xElement.className = 'neutral';
        }
        currentPositionMode2.x = x;
    }
    
    // Mettre à jour Y si fourni
    if (y !== null) {
        const yElement = mode2Page.querySelector('#coordYMode2 span');
        if (yElement) {
            yElement.textContent = (y >= 0 ? '+' : '') + y;
            if (y > 0) yElement.className = 'positive';
            else if (y < 0) yElement.className = 'negative';
            else yElement.className = 'neutral';
        }
        currentPositionMode2.y = y;
    }
    
    // Mettre à jour Z si fourni
    if (z !== null) {
        const zElement = mode2Page.querySelector('#coordZMode2 span');
        if (zElement) {
            zElement.textContent = (z >= 0 ? '+' : '') + z;
            if (z > 0) zElement.className = 'positive';
            else if (z < 0) zElement.className = 'negative';
            else zElement.className = 'neutral';
        }
        currentPositionMode2.z = z;
    }
    
    console.log('Coordonnées Mode 2 mises à jour - X:', currentPositionMode2.x, 'Y:', currentPositionMode2.y, 'Z:', currentPositionMode2.z);
}

// Fonction pour simuler l'intégration avec Sopha
function updateSophaVisualization(x, y, z) {
    console.log(`Mise à jour visualisation Sopha 3D: X:${x}, Y:${y}, Z:${z}`);
    
    // Ici vous intégrerez l'API Sopha pour la visualisation 3D
    // Exemple de ce qui pourrait être fait :
    /*
    if (window.SophaAPI) {
        window.SophaAPI.updateNeedlePosition({
            x: x,
            y: y,
            z: z,
            prostateModel: 'current_patient',
            realTime: true
        });
    }
    */
    
    // Pour l'instant, simulation avec log
    const prostateVisual = document.querySelector('#mode2 .prostate-visual');
    if (prostateVisual) {
        // Simuler un changement visuel
        prostateVisual.style.backgroundColor = `rgba(46, 109, 164, ${Math.abs(z) / 100 + 0.1})`;
        
        // Ajouter un indicateur de mise à jour
        const updateIndicator = prostateVisual.querySelector('.update-indicator');
        if (updateIndicator) {
            updateIndicator.remove();
        }
        
        const indicator = document.createElement('div');
        indicator.className = 'update-indicator';
        indicator.style.cssText = `
            position: absolute;
            top: 5px;
            right: 5px;
            background: #4caf50;
            color: white;
            padding: 2px 6px;
            border-radius: 3px;
            font-size: 10px;
            z-index: 10;
        `;
        indicator.textContent = 'Sopha ✓';
        prostateVisual.style.position = 'relative';
        prostateVisual.appendChild(indicator);
        
        // Supprimer l'indicateur après 2 secondes
        setTimeout(() => {
            if (indicator.parentNode) {
                indicator.remove();
            }
        }, 2000);
    }
}

// Fonction pour nettoyer la page lors du changement de page
function cleanupMode2Page() {
    console.log('Nettoyage de la page Mode 2');
    
    // Supprimer les écouteurs d'événements pour éviter les fuites de mémoire
    const mode2Page = document.getElementById('mode2');
    if (mode2Page) {
        // Nettoyer les joysticks
        const joysticks = mode2Page.querySelectorAll('.joystick');
        joysticks.forEach(joystick => {
            const arrows = joystick.querySelectorAll('.arrow');
            arrows.forEach(arrow => {
                const newArrow = arrow.cloneNode(true);
                arrow.parentNode.replaceChild(newArrow, arrow);
            });
        });
        
        // Nettoyer les boutons
        const buttons = mode2Page.querySelectorAll('button');
        buttons.forEach(button => {
            const newButton = button.cloneNode(true);
            button.parentNode.replaceChild(newButton, button);
        });
        
        // Nettoyer les inputs
        const inputs = mode2Page.querySelectorAll('input');
        inputs.forEach(input => {
            const newInput = input.cloneNode(true);
            input.parentNode.replaceChild(newInput, input);
        });
    }
}

// Exportation des fonctions pour utilisation dans le module principal
export {
    initMode2Page,
    updateNeedleCoordinatesMode2,
    updateSophaVisualization,
    cleanupMode2Page
};
