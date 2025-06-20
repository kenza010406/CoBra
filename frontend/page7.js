// page7.js - Page Admin (Nouvelle version)
// Gestion de la page de calibration administrateur du système CoBra

// Fonction d'initialisation de la page de calibration
function initCalibrationPage() {
    console.log('Initialisation de la page de calibration (nouvelle version)');
    
    // Trouver la div de calibration existante et la vider
    const calibrationContent = document.getElementById('calibration');
    
    // Si l'élément existe, on le vide complètement pour remplacer son contenu
    if (calibrationContent) {
        calibrationContent.innerHTML = '';
        
        // Ajouter le titre de la page
        const modeTitle = document.createElement('h2');
        modeTitle.className = 'page-title';
        modeTitle.textContent = 'Mode Administrateur';
        calibrationContent.appendChild(modeTitle);
        
        // Créer un conteneur flex pour organiser les sections
        const flexContainer = document.createElement('div');
        flexContainer.style.display = 'flex';
        flexContainer.style.flexWrap = 'wrap';
        flexContainer.style.gap = '10px';
        calibrationContent.appendChild(flexContainer);
        
        // Créer la colonne de gauche
        const leftColumn = document.createElement('div');
        leftColumn.style.flex = '1';
        leftColumn.style.minWidth = '45%';
        leftColumn.style.maxWidth = '48%';
        
        // Créer la colonne de droite
        const rightColumn = document.createElement('div');
        rightColumn.style.flex = '1';
        rightColumn.style.minWidth = '45%';
        rightColumn.style.maxWidth = '48%';
        
        // Ajouter Section 1 - Point Zéro (colonne gauche)
        const section1 = createSection('1', 'Point Zéro');
        
        // Ajouter la description
        const zeroDescription = document.createElement('div');
        zeroDescription.className = 'section-description';
        zeroDescription.textContent = 'Établit le point de référence à partir duquel tout le mouvement sont calculés.';
        section1.appendChild(zeroDescription);
        
        // Position actuelle comme zéro avec icône d'activation
        const positionRow = document.createElement('div');
        positionRow.className = 'parameter-row';
        
        const positionLabel = document.createElement('div');
        positionLabel.className = 'parameter-label';
        positionLabel.textContent = 'Position actuelle comme zéro :';
        
        // Créer l'icône d'activation (toggle bleu) au lieu d'une checkbox
        const toggleIcon = document.createElement('div');
        toggleIcon.className = 'toggle-icon active';
        toggleIcon.style.width = '40px';
        toggleIcon.style.height = '20px';
        toggleIcon.style.borderRadius = '10px';
        toggleIcon.style.backgroundColor = '#2196F3';
        toggleIcon.style.position = 'relative';
        toggleIcon.style.cursor = 'pointer';
        
        // Ajouter le cercle blanc sur le toggle
        const toggleCircle = document.createElement('div');
        toggleCircle.style.width = '16px';
        toggleCircle.style.height = '16px';
        toggleCircle.style.borderRadius = '50%';
        toggleCircle.style.backgroundColor = 'white';
        toggleCircle.style.position = 'absolute';
        toggleCircle.style.top = '2px';
        toggleCircle.style.right = '2px';
        toggleCircle.style.transition = '0.3s';
        
        toggleIcon.appendChild(toggleCircle);
        
        positionRow.appendChild(positionLabel);
        positionRow.appendChild(toggleIcon);
        section1.appendChild(positionRow);
        
        // Points de référence sauvegardés
        const pointsRow = document.createElement('div');
        pointsRow.className = 'parameter-row';
        
        const pointsLabel = document.createElement('div');
        pointsLabel.className = 'parameter-label';
        pointsLabel.textContent = 'Points de référence sauvegardés :';
        
        const pointsSelect = document.createElement('select');
        pointsSelect.className = 'parameter-select';
        pointsSelect.id = 'savedPointsSelect';
        pointsSelect.style.padding = '5px';
        pointsSelect.style.border = '1px solid #ccc';
        pointsSelect.style.borderRadius = '4px';
        
        const defaultOption = document.createElement('option');
        defaultOption.value = 'default';
        defaultOption.textContent = 'Point par défaut';
        pointsSelect.appendChild(defaultOption);
        
        pointsRow.appendChild(pointsLabel);
        pointsRow.appendChild(pointsSelect);
        section1.appendChild(pointsRow);
        
        leftColumn.appendChild(section1);
        
        // Ajouter Section 3 - Prévention Collisions (colonne gauche)
        const section3 = createSection('3', 'Prévention Collisions');
        
        // Protection collision avec icône d'activation
        const collisionRow = document.createElement('div');
        collisionRow.className = 'parameter-row';
        
        const collisionLabel = document.createElement('div');
        collisionLabel.className = 'parameter-label';
        collisionLabel.textContent = 'Activer protection collision :';
        
        // Créer l'icône d'activation (toggle bleu) au lieu d'une checkbox
        const collisionToggle = document.createElement('div');
        collisionToggle.className = 'toggle-icon active';
        collisionToggle.style.width = '40px';
        collisionToggle.style.height = '20px';
        collisionToggle.style.borderRadius = '10px';
        collisionToggle.style.backgroundColor = '#2196F3';
        collisionToggle.style.position = 'relative';
        collisionToggle.style.cursor = 'pointer';
        
        // Ajouter le cercle blanc sur le toggle
        const collisionCircle = document.createElement('div');
        collisionCircle.style.width = '16px';
        collisionCircle.style.height = '16px';
        collisionCircle.style.borderRadius = '50%';
        collisionCircle.style.backgroundColor = 'white';
        collisionCircle.style.position = 'absolute';
        collisionCircle.style.top = '2px';
        collisionCircle.style.right = '2px';
        collisionCircle.style.transition = '0.3s';
        
        collisionToggle.appendChild(collisionCircle);
        
        collisionRow.appendChild(collisionLabel);
        collisionRow.appendChild(collisionToggle);
        section3.appendChild(collisionRow);
        
        // Marge de sécurité
        const marginRow = document.createElement('div');
        marginRow.className = 'parameter-row';
        
        const marginLabel = document.createElement('div');
        marginLabel.className = 'parameter-label';
        marginLabel.textContent = 'Marge de sécurité :';
        
        // Créer un conteneur pour l'input et l'unité
        const marginInputContainer = document.createElement('div');
        marginInputContainer.style.display = 'flex';
        marginInputContainer.style.alignItems = 'center';
        
        const marginInput = document.createElement('input');
        marginInput.type = 'text';
        marginInput.className = 'parameter-input small-input';
        marginInput.value = '5';
        marginInput.style.width = '30px';
        marginInput.style.textAlign = 'center';
        marginInput.style.marginRight = '5px';
        
        const marginUnit = document.createElement('span');
        marginUnit.className = 'parameter-unit';
        marginUnit.textContent = 'mm';
        
        marginInputContainer.appendChild(marginInput);
        marginInputContainer.appendChild(marginUnit);
        
        marginRow.appendChild(marginLabel);
        marginRow.appendChild(marginInputContainer);
        section3.appendChild(marginRow);
        
        leftColumn.appendChild(section3);
        
        // Ajouter Section 5 - Capteurs (colonne gauche)
        const section5 = createSection('5', 'Capteurs');
        
        // Conteneur pour les capteurs (style tableau)
        const sensorsTable = document.createElement('div');
        sensorsTable.style.display = 'flex';
        sensorsTable.style.justifyContent = 'space-between';
        sensorsTable.style.marginBottom = '10px';
        
        const sensorIds = ['M1', 'M2', 'M3', 'M4', 'M5'];
        const sensorClasses = ['green', 'green', 'green', 'green', 'orange']; // Classes pour chaque capteur
        
        sensorIds.forEach((id, index) => {
            const sensorCell = document.createElement('div');
            sensorCell.style.textAlign = 'center';
            sensorCell.style.padding = '5px';
            
            const sensorLabel = document.createElement('div');
            sensorLabel.textContent = id;
            sensorLabel.style.marginBottom = '5px';
            sensorLabel.style.fontWeight = 'normal';
            
            const sensorStatus = document.createElement('div');
            sensorStatus.style.width = '15px';
            sensorStatus.style.height = '15px';
            sensorStatus.style.borderRadius = '50%';
            sensorStatus.style.margin = '0 auto';
            
            // Définir la couleur du capteur
            if (sensorClasses[index] === 'green') {
                sensorStatus.style.backgroundColor = '#4caf50';
            } else if (sensorClasses[index] === 'orange') {
                sensorStatus.style.backgroundColor = '#ff9800';
            } else {
                sensorStatus.style.backgroundColor = '#aaa';
            }
            
            sensorCell.appendChild(sensorLabel);
            sensorCell.appendChild(sensorStatus);
            
            sensorsTable.appendChild(sensorCell);
        });
        
        section5.appendChild(sensorsTable);
        
        // Message pour M5 non calibré
        const sensorMessage = document.createElement('div');
        sensorMessage.className = 'sensor-message';
        sensorMessage.textContent = 'Moteur M5 non calibré';
        sensorMessage.style.fontSize = '12px';
        sensorMessage.style.color = '#666';
        sensorMessage.style.fontStyle = 'italic';
        sensorMessage.style.textAlign = 'center';
        sensorMessage.style.marginBottom = '10px';
        section5.appendChild(sensorMessage);
        
        // Bouton pour tout calibrer
        const calibrateAllButton = document.createElement('button');
        calibrateAllButton.className = 'calibrate-all-button';
        calibrateAllButton.textContent = 'Tout calibrer';
        calibrateAllButton.style.display = 'block';
        calibrateAllButton.style.margin = '0 auto';
        calibrateAllButton.style.padding = '5px 15px';
        calibrateAllButton.style.backgroundColor = 'white';
        calibrateAllButton.style.border = '1px solid #ccc';
        calibrateAllButton.style.borderRadius = '4px';
        calibrateAllButton.style.cursor = 'pointer';
        section5.appendChild(calibrateAllButton);
        
        leftColumn.appendChild(section5);
        
        // Ajouter Section 2 - Espace de travail (colonne droite)
        const section2 = createSection('2', 'Espace de travail');
        
        // Créer les champs pour X, Y, Z min/max
        const axisLabels = ['X min:', 'X max:', 'Y min:', 'Y max:', 'Z min:', 'Z max:'];
        const workspaceContainer = document.createElement('div');
        workspaceContainer.style.display = 'grid';
        workspaceContainer.style.gridTemplateColumns = '1fr';
        workspaceContainer.style.gap = '10px';
        
        axisLabels.forEach(label => {
            const row = document.createElement('div');
            row.style.display = 'flex';
            row.style.alignItems = 'center';
            row.style.justifyContent = 'space-between';
            
            const labelElement = document.createElement('div');
            labelElement.textContent = label;
            labelElement.style.fontSize = '14px';
            labelElement.style.width = '60px';
            
            const controlsElement = document.createElement('div');
            controlsElement.style.display = 'flex';
            controlsElement.style.alignItems = 'center';
            
            const minusButton = document.createElement('button');
            minusButton.textContent = '-';
            minusButton.style.width = '30px';
            minusButton.style.height = '30px';
            minusButton.style.backgroundColor = '#2e6da4';
            minusButton.style.color = 'white';
            minusButton.style.border = 'none';
            minusButton.style.borderRadius = '4px';
            minusButton.style.fontSize = '16px';
            minusButton.style.cursor = 'pointer';
            
            const valueInput = document.createElement('input');
            valueInput.type = 'text';
            valueInput.value = '-50';
            valueInput.style.width = '50px';
            valueInput.style.textAlign = 'center';
            valueInput.style.margin = '0 5px';
            valueInput.style.padding = '5px';
            valueInput.style.border = '1px solid #ccc';
            valueInput.style.borderRadius = '4px';
            
            const plusButton = document.createElement('button');
            plusButton.textContent = '+';
            plusButton.style.width = '30px';
            plusButton.style.height = '30px';
            plusButton.style.backgroundColor = '#2e6da4';
            plusButton.style.color = 'white';
            plusButton.style.border = 'none';
            plusButton.style.borderRadius = '4px';
            plusButton.style.fontSize = '16px';
            plusButton.style.cursor = 'pointer';
            
            controlsElement.appendChild(minusButton);
            controlsElement.appendChild(valueInput);
            controlsElement.appendChild(plusButton);
            
            row.appendChild(labelElement);
            row.appendChild(controlsElement);
            
            workspaceContainer.appendChild(row);
        });
        
        section2.appendChild(workspaceContainer);
        rightColumn.appendChild(section2);
        
        // Ajouter Section 4 - Précision Millimétrique (colonne droite)
        const section4 = createSection('4', 'Précision Millimétrique');
        
        // Précision ciblée
        const precisionRow = document.createElement('div');
        precisionRow.style.marginBottom = '10px';
        
        const precisionLabel = document.createElement('div');
        precisionLabel.textContent = 'Précision ciblée:';
        precisionLabel.style.marginBottom = '5px';
        
        const precisionValueContainer = document.createElement('div');
        precisionValueContainer.style.display = 'flex';
        precisionValueContainer.style.alignItems = 'center';
        
        const precisionValue = document.createElement('span');
        precisionValue.id = 'precision-value-display'; // Ajout d'un ID unique
        precisionValue.textContent = '0.1';
        precisionValue.style.fontWeight = 'bold';
        const precisionUnit = document.createElement('span');
        precisionUnit.textContent = ' mm';
        precisionUnit.style.marginLeft = '5px';
        
        precisionValueContainer.appendChild(precisionValue);
        precisionValueContainer.appendChild(precisionUnit);
        
        precisionRow.appendChild(precisionLabel);
        precisionRow.appendChild(precisionValueContainer);
        section4.appendChild(precisionRow);
        
        // Boutons de précision
        const precisionButtons = document.createElement('div');
        precisionButtons.style.display = 'flex';
        precisionButtons.style.justifyContent = 'space-between';
        
        const precisionValues = ['0.01', '0.1', '0.5', '1.0'];
        
        precisionValues.forEach(value => {
            const button = document.createElement('button');
            button.textContent = value;
            button.style.flex = '1';
            button.style.padding = '8px';
            button.style.margin = '0 5px';
            button.style.border = '1px solid #ccc';
            button.style.borderRadius = '4px';
            button.style.cursor = 'pointer';
            
            // Marquer le bouton actif
            if (value === '0.1') {
                button.style.backgroundColor = '#2e6da4';
                button.style.color = 'white';
                button.style.borderColor = '#2e6da4';
            } else {
                button.style.backgroundColor = 'white';
                button.style.color = 'black';
            }
            
            precisionButtons.appendChild(button);
        });
        
        section4.appendChild(precisionButtons);
        rightColumn.appendChild(section4);
        
        // Ajouter les colonnes au conteneur flex
        flexContainer.appendChild(leftColumn);
        flexContainer.appendChild(rightColumn);
        
        // Conteneur pour le bouton Lancer (centré)
        const launchButtonContainer = document.createElement('div');
        launchButtonContainer.style.display = 'flex';
        launchButtonContainer.style.justifyContent = 'center';
        launchButtonContainer.style.marginTop = '20px';
        
        // Bouton Lancer
        const launchButton = document.createElement('button');
        launchButton.textContent = 'Lancer';
        launchButton.id = 'launchButton';
        launchButton.style.padding = '12px 30px';
        launchButton.style.backgroundColor = '#00c853';
        launchButton.style.color = 'white';
        launchButton.style.border = 'none';
        launchButton.style.borderRadius = '4px';
        launchButton.style.fontSize = '18px';
        launchButton.style.cursor = 'pointer';
        
        launchButtonContainer.appendChild(launchButton);
        calibrationContent.appendChild(launchButtonContainer);
    }
    
    // Enregistrer les gestionnaires d'événements
    setupCalibrationEventListeners();
    
    return calibrationContent;
}

// Fonction pour créer une section avec un numéro et un titre
function createSection(number, title) {
    const section = document.createElement('div');
    section.style.backgroundColor = '#ecf0f3';
    section.style.borderRadius = '8px';
    section.style.marginBottom = '15px';
    section.style.padding = '10px 15px';
    section.style.width = '100%';
    
    const header = document.createElement('div');
    header.style.display = 'flex';
    header.style.alignItems = 'center';
    header.style.marginBottom = '10px';
    header.style.borderBottom = '1px solid #ccc';
    header.style.paddingBottom = '5px';
    
    const numberCircle = document.createElement('div');
    numberCircle.style.backgroundColor = '#2e6da4';
    numberCircle.style.color = 'white';
    numberCircle.style.width = '25px';
    numberCircle.style.height = '25px';
    numberCircle.style.borderRadius = '50%';
    numberCircle.style.display = 'flex';
    numberCircle.style.justifyContent = 'center';
    numberCircle.style.alignItems = 'center';
    numberCircle.style.fontWeight = 'bold';
    numberCircle.style.marginRight = '10px';
    numberCircle.textContent = number;
    
    const titleElement = document.createElement('div');
    titleElement.style.color = '#2e6da4';
    titleElement.style.fontSize = '16px';
    titleElement.style.fontWeight = 'bold';
    titleElement.textContent = title;
    
    header.appendChild(numberCircle);
    header.appendChild(titleElement);
    
    section.appendChild(header);
    
    return section;
}

// Configuration des écouteurs d'événements pour la page de calibration
function setupCalibrationEventListeners() {
    console.log('Configuration des écouteurs d\'événements pour la page de calibration');
    
    // Gestionnaire pour les boutons d'espace de travail (+ et -)
     document.querySelectorAll('button').forEach(button => {
        if (button.textContent === '+' || button.textContent === '-') {
            button.addEventListener('click', function() {
                const input = this.parentNode.querySelector('input');
                let value = parseInt(input.value);
                
                if (this.textContent === '+') {
                    value += 10;
                } else {
                    value -= 10;
                }
                
                input.value = value;
            });
        }
    });
    
    // Gestionnaire pour les boutons de précision
    const precisionButtons = document.querySelectorAll('[style*="padding: 8px"][style*="border: 1px solid"]');
    
    if (precisionButtons.length > 0) {
        precisionButtons.forEach(button => {
            button.addEventListener('click', function() {
                // Utiliser l'ID pour trouver directement l'élément d'affichage
                const precisionValueDisplay = document.getElementById('precision-value-display');
                
                if (precisionValueDisplay) {
                    // Changer la valeur affichée
                    precisionValueDisplay.textContent = this.textContent;
                    console.log('Précision mise à jour:', this.textContent);
                } else {
                    console.error('Élément d\'affichage de précision non trouvé');
                }
                
                // Mettre à jour l'apparence des boutons
                precisionButtons.forEach(btn => {
                    btn.style.backgroundColor = 'white';
                    btn.style.color = 'black';
                    btn.style.borderColor = '#ccc';
                });
                
                this.style.backgroundColor = '#2e6da4';
                this.style.color = 'white';
                this.style.borderColor = '#2e6da4';
            });
        });
    } else {
        console.error('Boutons de précision non trouvés');
    }
    
    // Gestionnaire pour les toggle switches
    document.querySelectorAll('.toggle-icon').forEach(toggle => {
        toggle.addEventListener('click', function() {
            this.classList.toggle('active');
            
            if (this.classList.contains('active')) {
                this.style.backgroundColor = '#2196F3';
                this.querySelector('div').style.right = '2px';
                this.querySelector('div').style.left = 'auto';
            } else {
                this.style.backgroundColor = '#ccc';
                this.querySelector('div').style.right = 'auto';
                this.querySelector('div').style.left = '2px';
            }
        });
    });
    
    // Gestionnaire pour le bouton "Tout calibrer"
     const calibrateAllButton = document.querySelector('.calibrate-all-button');
    if (calibrateAllButton) {
        calibrateAllButton.addEventListener('click', function() {
            // Simuler une calibration
            alert('Calibration de tous les moteurs en cours...');
            
            // Mettre à jour les statuts des capteurs (tous en vert)
            const sensorStatuses = document.querySelectorAll('[style*="width: 15px"][style*="height: 15px"][style*="border-radius: 50%"]');
            sensorStatuses.forEach(status => {
                status.style.backgroundColor = '#4caf50';
            });
            
            // Mettre à jour le message
            const sensorMessage = document.querySelector('.sensor-message');
            if (sensorMessage) {
                sensorMessage.textContent = 'Tous les moteurs calibrés';
            } else {
                console.error('Élément de message de capteur non trouvé');
            }
        });
    } else {
        console.error('Bouton de calibration non trouvé');
    }
    // Gestionnaire pour le bouton "Lancer"
    const launchButton = document.getElementById('launchButton');
    if (launchButton) {
        launchButton.addEventListener('click', function() {
            alert('Système lancé avec les paramètres configurés !');
        });
    }
}

// Fonction pour nettoyer la page lors du changement de page
function cleanupCalibrationPage() {
    console.log('Nettoyage de la page de calibration');
    
    // Supprimer les écouteurs d'événements pour éviter les fuites de mémoire
    document.querySelectorAll('button').forEach(button => {
        if (button.textContent === '+' || button.textContent === '-') {
            button.removeEventListener('click', null);
        }
    });
    
    document.querySelectorAll('.toggle-icon').forEach(toggle => {
        toggle.removeEventListener('click', null);
    });
    
    const calibrateAllButton = document.querySelector('button[textContent="Tout calibrer"]');
    if (calibrateAllButton) {
        calibrateAllButton.removeEventListener('click', null);
    }
    
    const launchButton = document.getElementById('launchButton');
    if (launchButton) {
        launchButton.removeEventListener('click', null);
    }
}

// Exportation des fonctions pour utilisation dans le module principal
export {
    initCalibrationPage,
    cleanupCalibrationPage
};
