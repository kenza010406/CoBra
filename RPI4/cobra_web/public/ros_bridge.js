// ros_bridge.js - DEBUG WORKSPACE 
console.log('🚀 ROS Bridge - DEBUG WORKSPACE ');

const ROSBRIDGE_URL = 'ws://localhost:9090';
let ros;
let isConnected = false;
let commandPublisher;
let statusSubscriber;

// =================================================================
// ÉTAT LOCAL AVEC DÉBOGAGE
// =================================================================
let currentWorkspaceLimits = {
    x_min: -50.0,
    x_max: 50.0,
    y_min: -50.0,
    y_max: 50.0,
    z_min: -50.0,
    z_max: 50.0
};

let currentPosition = {
    x: 0.0,
    y: 0.0,
    z: 0.0
};

let stepSize = 5.0;

// =================================================================
// INITIALISATION ROS (IDENTIQUE)
// =================================================================
function initROS() {
    console.log('📡 Connexion ROSBridge...');
    
    ros = new ROSLIB.Ros({ url: ROSBRIDGE_URL });

    ros.on('connection', function() {
        console.log('✅ ROSBridge connecté !');
        isConnected = true;
        setupCommunication();
    });

    ros.on('error', function(error) {
        console.log('❌ Erreur ROSBridge:', error);
        isConnected = false;
    });

    ros.on('close', function() {
        console.log('🔌 Connexion ROSBridge fermée');
        isConnected = false;
    });
}

function setupCommunication() {
    commandPublisher = new ROSLIB.Topic({
        ros: ros,
        name: '/rpi4_to_rpi5',
        messageType: 'std_msgs/msg/String'
    });
    
    statusSubscriber = new ROSLIB.Topic({
        ros: ros,
        name: '/rpi5_to_rpi4',
        messageType: 'std_msgs/msg/String'
    });
    
    statusSubscriber.subscribe(function(msg) {
        handleRpi5Message(msg.data);
    });
    
    console.log('📡 Communication RPi4↔RPi5 configurée');
    
    setTimeout(() => {
        debugWorkspaceInterface();
        syncWorkspaceLimitsFromInterface();
        sendToRpi5('TEST_AUTO_CONNECTION');
    }, 2000);
}

// =================================================================
// DÉBOGAGE DE L'INTERFACE
// =================================================================
function debugWorkspaceInterface() {
    console.log('🔍 === DÉBOGAGE INTERFACE WORKSPACE ===');
    
    // Chercher TOUS les inputs numériques
    const allInputs = document.querySelectorAll('input[type="number"]');
    console.log(`📊 ${allInputs.length} inputs numériques trouvés`);
    
    allInputs.forEach((input, index) => {
        const value = input.value;
        const id = input.id || 'sans-id';
        const name = input.name || 'sans-name';
        
        // Analyser le contexte
        const container = input.closest('div, label, td, tr');
        const labelElement = container ? container.querySelector('label, span, div') : null;
        const labelText = labelElement ? labelElement.textContent : 'sans-label';
        
        console.log(`🔍 Input ${index}:`);
        console.log(`   ID: "${id}"`);
        console.log(`   Name: "${name}"`);
        console.log(`   Valeur: "${value}"`);
        console.log(`   Label: "${labelText}"`);
        console.log(`   Classe: "${input.className}"`);
        
        // Tenter de détecter l'axe
        const axisName = getWorkspaceAxisDebug(input);
        console.log(`   Axe détecté: "${axisName}"`);
        console.log('   ---');
    });
    
    console.log('🔍 === FIN DÉBOGAGE ===');
}

function getWorkspaceAxisDebug(input) {
    console.log('🔍 Analyse axe pour:', input);
    
    // Méthode 1: Par ID
    const id = (input.id || '').toLowerCase();
    console.log(`   ID: "${id}"`);
    
    if (id.includes('xmin') || id.includes('x_min')) return 'X_MIN';
    if (id.includes('xmax') || id.includes('x_max')) return 'X_MAX';
    if (id.includes('ymin') || id.includes('y_min')) return 'Y_MIN';
    if (id.includes('ymax') || id.includes('y_max')) return 'Y_MAX';
    if (id.includes('zmin') || id.includes('z_min')) return 'Z_MIN';
    if (id.includes('zmax') || id.includes('z_max')) return 'Z_MAX';
    
    // Méthode 2: Par name
    const name = (input.name || '').toLowerCase();
    console.log(`   Name: "${name}"`);
    
    if (name.includes('xmin') || name.includes('x_min')) return 'X_MIN';
    if (name.includes('xmax') || name.includes('x_max')) return 'X_MAX';
    if (name.includes('ymin') || name.includes('y_min')) return 'Y_MIN';
    if (name.includes('ymax') || name.includes('y_max')) return 'Y_MAX';
    if (name.includes('zmin') || name.includes('z_min')) return 'Z_MIN';
    if (name.includes('zmax') || name.includes('z_max')) return 'Z_MAX';
    
    // Méthode 3: Par contexte (élargie)
    const contexts = [
        input.closest('div'),
        input.closest('label'),
        input.closest('td'),
        input.closest('tr'),
        input.parentNode,
        input.parentNode ? input.parentNode.parentNode : null
    ].filter(el => el);
    
    for (let context of contexts) {
        const allText = context.textContent.toLowerCase();
        console.log(`   Contexte: "${allText}"`);
        
        // X
        if (allText.includes('x')) {
            if (allText.includes('min') || allText.includes('minimum')) return 'X_MIN';
            if (allText.includes('max') || allText.includes('maximum')) return 'X_MAX';
        }
        
        // Y
        if (allText.includes('y')) {
            if (allText.includes('min') || allText.includes('minimum')) return 'Y_MIN';
            if (allText.includes('max') || allText.includes('maximum')) return 'Y_MAX';
        }
        
        // Z
        if (allText.includes('z')) {
            if (allText.includes('min') || allText.includes('minimum')) return 'Z_MIN';
            if (allText.includes('max') || allText.includes('maximum')) return 'Z_MAX';
        }
    }
    
    console.log('   ❌ Aucun axe détecté');
    return 'UNKNOWN';
}

// =================================================================
// SYNCHRONISATION AMÉLIORÉE
// =================================================================
function syncWorkspaceLimitsFromInterface() {
    console.log('🔄 Synchronisation limites workspace...');
    
    const workspaceInputs = document.querySelectorAll('input[type="number"]');
    let limitsFound = 0;
    
    workspaceInputs.forEach(input => {
        const axisName = getWorkspaceAxisDebug(input);
        const value = parseFloat(input.value) || 0;
        
        if (axisName !== 'UNKNOWN') {
            const axisKey = axisName.toLowerCase();
            if (axisKey in currentWorkspaceLimits) {
                currentWorkspaceLimits[axisKey] = value;
                console.log(`✅ ${axisName}: ${value}`);
                limitsFound++;
                
                // Envoyer au RPi5
                sendToRpi5(`WORKSPACE:${axisName}:${value}`);
            }
        }
    });
    
    console.log(`📊 ${limitsFound} limites trouvées et synchronisées`);
    console.log('📐 Limites actuelles:');
    console.log(`   X: ${currentWorkspaceLimits.x_min} à ${currentWorkspaceLimits.x_max}`);
    console.log(`   Y: ${currentWorkspaceLimits.y_min} à ${currentWorkspaceLimits.y_max}`);
    console.log(`   Z: ${currentWorkspaceLimits.z_min} à ${currentWorkspaceLimits.z_max}`);
    
    if (limitsFound === 0) {
        console.log('⚠️ AUCUNE LIMITE TROUVÉE - Utilisation des sélecteurs manuels');
        tryManualSelectors();
    }
}

function tryManualSelectors() {
    console.log('🔍 Tentative sélecteurs manuels...');
    
    // Liste de sélecteurs possibles
    const selectors = [
        '#xmin, #x_min, #workspace_x_min',
        '#xmax, #x_max, #workspace_x_max',
        '#ymin, #y_min, #workspace_y_min',
        '#ymax, #y_max, #workspace_y_max',
        '#zmin, #z_min, #workspace_z_min',
        '#zmax, #z_max, #workspace_z_max',
        'input[name*="xmin"], input[name*="x_min"]',
        'input[name*="xmax"], input[name*="x_max"]',
        'input[name*="ymin"], input[name*="y_min"]',
        'input[name*="ymax"], input[name*="y_max"]',
        'input[name*="zmin"], input[name*="z_min"]',
        'input[name*="zmax"], input[name*="z_max"]'
    ];
    
    const axes = ['X_MIN', 'X_MAX', 'Y_MIN', 'Y_MAX', 'Z_MIN', 'Z_MAX'];
    
    selectors.forEach((selector, index) => {
        const element = document.querySelector(selector);
        if (element) {
            const value = parseFloat(element.value) || 0;
            const axisName = axes[index % 6];
            const axisKey = axisName.toLowerCase();
            
            if (axisKey in currentWorkspaceLimits) {
                currentWorkspaceLimits[axisKey] = value;
                console.log(`✅ Manuel ${axisName}: ${value} (sélecteur: ${selector})`);
                sendToRpi5(`WORKSPACE:${axisName}:${value}`);
            }
        }
    });
}

// =================================================================
// ENVOI ET RÉCEPTION (IDENTIQUES)
// =================================================================
function sendToRpi5(command) {
    if (!ros || !isConnected || !commandPublisher) {
        console.log('❌ Pas de connexion ROS');
        return false;
    }

    try {
        const message = new ROSLIB.Message({ data: command });
        commandPublisher.publish(message);
        console.log(`✅ ENVOYÉ: ${command}`);
        return true;
    } catch (error) {
        console.log('❌ Erreur envoi:', error);
        return false;
    }
}

function handleRpi5Message(data) {
    try {
        const status = JSON.parse(data);
        
        // Mettre à jour position locale AVEC DÉBOGAGE
        if (status.position) {
            console.log(`📍 Position reçue du RPi5: X=${status.position.x}, Y=${status.position.y}, Z=${status.position.z}`);
            currentPosition.x = status.position.x;
            currentPosition.y = status.position.y;
            currentPosition.z = status.position.z;
        }
        
        if (status.workspace_limits) {
            console.log('📐 Limites workspace reçues du RPi5:', status.workspace_limits);
            currentWorkspaceLimits = status.workspace_limits;
        }
        
    } catch (e) {
        console.log('📝 Message simple:', data);
    }
}

// =================================================================
// VÉRIFICATION AVEC DÉBOGAGE COMPLET
// =================================================================
function checkMovementAllowed(axisType, direction) {
    console.log(`🔍 === VÉRIFICATION MOUVEMENT ===`);
    console.log(`   Axe: ${axisType}, Direction: ${direction}`);
    console.log(`   Position actuelle: X=${currentPosition.x}, Y=${currentPosition.y}, Z=${currentPosition.z}`);
    console.log(`   Step size: ${stepSize}`);
    
    // Calculer nouvelle position
    let newPosition = { ...currentPosition };
    
    if (axisType === 'XY') {
        if (direction === 'UP') {
            newPosition.y += stepSize;
            console.log(`   Nouveau Y: ${currentPosition.y} + ${stepSize} = ${newPosition.y}`);
        } else if (direction === 'DOWN') {
            newPosition.y -= stepSize;
            console.log(`   Nouveau Y: ${currentPosition.y} - ${stepSize} = ${newPosition.y}`);
        } else if (direction === 'LEFT') {
            newPosition.x -= stepSize;
            console.log(`   Nouveau X: ${currentPosition.x} - ${stepSize} = ${newPosition.x}`);
        } else if (direction === 'RIGHT') {
            newPosition.x += stepSize;
            console.log(`   Nouveau X: ${currentPosition.x} + ${stepSize} = ${newPosition.x}`);
        }
    } else if (axisType === 'Z') {
        if (direction === 'UP') {
            newPosition.z += stepSize;
            console.log(`   Nouveau Z: ${currentPosition.z} + ${stepSize} = ${newPosition.z}`);
        } else if (direction === 'DOWN') {
            newPosition.z -= stepSize;
            console.log(`   Nouveau Z: ${currentPosition.z} - ${stepSize} = ${newPosition.z}`);
        }
    }
    
    console.log(`   Position calculée: X=${newPosition.x}, Y=${newPosition.y}, Z=${newPosition.z}`);
    console.log(`   Limites actuelles:`);
    console.log(`     X: [${currentWorkspaceLimits.x_min}, ${currentWorkspaceLimits.x_max}]`);
    console.log(`     Y: [${currentWorkspaceLimits.y_min}, ${currentWorkspaceLimits.y_max}]`);
    console.log(`     Z: [${currentWorkspaceLimits.z_min}, ${currentWorkspaceLimits.z_max}]`);
    
    // Vérifier chaque axe
    const xOk = (newPosition.x >= currentWorkspaceLimits.x_min && newPosition.x <= currentWorkspaceLimits.x_max);
    const yOk = (newPosition.y >= currentWorkspaceLimits.y_min && newPosition.y <= currentWorkspaceLimits.y_max);
    const zOk = (newPosition.z >= currentWorkspaceLimits.z_min && newPosition.z <= currentWorkspaceLimits.z_max);
    
    console.log(`   Vérifications:`);
    console.log(`     X OK: ${xOk} (${newPosition.x} ∈ [${currentWorkspaceLimits.x_min}, ${currentWorkspaceLimits.x_max}])`);
    console.log(`     Y OK: ${yOk} (${newPosition.y} ∈ [${currentWorkspaceLimits.y_min}, ${currentWorkspaceLimits.y_max}])`);
    console.log(`     Z OK: ${zOk} (${newPosition.z} ∈ [${currentWorkspaceLimits.z_min}, ${currentWorkspaceLimits.z_max}])`);
    
    const allowed = xOk && yOk && zOk;
    console.log(`   RÉSULTAT: ${allowed ? '✅ AUTORISÉ' : '🚫 BLOQUÉ'}`);
    console.log(`🔍 === FIN VÉRIFICATION ===`);
    
    return allowed;
}

// =================================================================
// JOYSTICKS AVEC DÉBOGAGE
// =================================================================
function setupJoystickControls() {
    console.log('🎮 Configuration joysticks avec vérification complète...');
    
    setupSingleJoystickWithLimits('joystick1', 'MODE1', 'XY');
    setupSingleJoystickWithLimits('joystick2', 'MODE1', 'Z');
    setupSingleJoystickWithLimits('joystick1Mode2', 'MODE2', 'XY');
    setupSingleJoystickWithLimits('joystick2Mode2', 'MODE2', 'Z');
}

function setupSingleJoystickWithLimits(joystickId, mode, axisType) {
    const joystick = document.getElementById(joystickId);
    if (!joystick) {
        console.log(`⚠️ Joystick ${joystickId} non trouvé`);
        return;
    }
    
    console.log(`🎮 Configuration ${joystickId} → ${mode} ${axisType}`);
    
    const arrows = joystick.querySelectorAll('.arrow, .arrow-up, .arrow-down, .arrow-left, .arrow-right, [class*="arrow"]');
    
    arrows.forEach(arrow => {
        arrow.addEventListener('mousedown', function(e) {
            e.preventDefault();
            
            let direction = '';
            const classes = Array.from(this.classList);
            
            if (classes.some(c => c.includes('up'))) direction = 'UP';
            else if (classes.some(c => c.includes('down'))) direction = 'DOWN';
            else if (classes.some(c => c.includes('left'))) direction = 'LEFT';
            else if (classes.some(c => c.includes('right'))) direction = 'RIGHT';
            
            if (direction) {
                console.log(`🎮 CLIC ${joystickId} ${direction}`);
                
                // VÉRIFICATION AVEC DÉBOGAGE COMPLET
                if (checkMovementAllowed(axisType, direction)) {
                    const command = `JOYSTICK:${mode}:${axisType}:${direction}`;
                    console.log(`✅ Envoi: ${command}`);
                    sendToRpi5(command);
                } else {
                    console.log(`🚫 Mouvement ${direction} BLOQUÉ`);
                    showVisualWarning(`Mouvement ${direction} bloqué - Limite atteinte`);
                }
            }
        });
    });
    
    console.log(`✅ ${joystickId} configuré (${arrows.length} flèches)`);
}

// =================================================================
// WORKSPACE AVEC SURVEILLANCE ÉTENDUE
// =================================================================
function setupWorkspaceControls() {
    console.log('📐 Configuration workspace avec surveillance étendue...');
    
    // Surveillance de TOUS les inputs numériques
    const allInputs = document.querySelectorAll('input[type="number"]');
    
    allInputs.forEach(input => {
        // Ajouter surveillance changement
        input.addEventListener('change', function() {
            console.log(`🔄 Input changé: ${this.value}`);
            const axisName = getWorkspaceAxisDebug(this);
            
            if (axisName !== 'UNKNOWN') {
                const value = parseFloat(this.value) || 0;
                updateLocalWorkspaceLimit(axisName, value);
                sendToRpi5(`WORKSPACE:${axisName}:${value}`);
            }
        });
        
        // Ajouter surveillance en temps réel
        input.addEventListener('input', function() {
            const axisName = getWorkspaceAxisDebug(this);
            if (axisName !== 'UNKNOWN') {
                const value = parseFloat(this.value) || 0;
                console.log(`⚡ Saisie en cours ${axisName}: ${value}`);
            }
        });
    });
    
    // Boutons +/-
    const allButtons = document.querySelectorAll('button');
    allButtons.forEach(btn => {
        if (btn.textContent === '+' || btn.textContent === '-') {
            btn.addEventListener('click', function(e) {
                e.preventDefault();
                
                const input = this.parentNode.querySelector('input[type="number"]');
                if (!input) return;
                
                const operation = this.textContent;
                const axisName = getWorkspaceAxisDebug(input);
                
                if (axisName !== 'UNKNOWN') {
                    let currentValue = parseFloat(input.value) || 0;
                    if (operation === '+') currentValue += 10;
                    else currentValue -= 10;
                    
                    input.value = currentValue;
                    updateLocalWorkspaceLimit(axisName, currentValue);
                    sendToRpi5(`WORKSPACE:${axisName}:${currentValue}`);
                }
            });
        }
    });
}

function updateLocalWorkspaceLimit(axisName, value) {
    const axisKey = axisName.toLowerCase();
    if (axisKey in currentWorkspaceLimits) {
        const oldValue = currentWorkspaceLimits[axisKey];
        currentWorkspaceLimits[axisKey] = value;
        console.log(`🔄 ${axisName}: ${oldValue} → ${value}`);
        console.log('📐 Nouvelles limites:');
        console.log(`   X: [${currentWorkspaceLimits.x_min}, ${currentWorkspaceLimits.x_max}]`);
        console.log(`   Y: [${currentWorkspaceLimits.y_min}, ${currentWorkspaceLimits.y_max}]`);
        console.log(`   Z: [${currentWorkspaceLimits.z_min}, ${currentWorkspaceLimits.z_max}]`);
    }
}

function showVisualWarning(message) {
    const warning = document.createElement('div');
    warning.style.cssText = `
        position: fixed;
        top: 50%;
        left: 50%;
        transform: translate(-50%, -50%);
        background: #ff5722;
        color: white;
        padding: 20px;
        border-radius: 10px;
        font-weight: bold;
        z-index: 10000;
        text-align: center;
        box-shadow: 0 4px 20px rgba(0,0,0,0.3);
    `;
    warning.textContent = message;
    document.body.appendChild(warning);
    
    setTimeout(() => {
        if (warning.parentNode) {
            warning.parentNode.removeChild(warning);
        }
    }, 3000);
}

// =================================================================
// AUTRES CONTRÔLES (SIMPLIFIÉS POUR DÉBOGAGE)
// =================================================================
function setupModeControls() {
    const allButtons = document.querySelectorAll('button, .btn, input[type="button"], input[type="submit"]');
    
    allButtons.forEach((btn) => {
        const text = (btn.textContent || btn.value || '').toLowerCase();
        
        if (text.includes('mode 1') || text.includes('mode1')) {
            btn.addEventListener('click', function(e) {
                e.preventDefault();
                sendToRpi5('MODE:1');
            });
        }
        
        if (text.includes('mode 2') || text.includes('mode2')) {
            btn.addEventListener('click', function(e) {
                e.preventDefault();
                sendToRpi5('MODE:2');
            });
        }
    });
}

function setupSystemControls() {
    const allButtons = document.querySelectorAll('button, .btn, input[type="button"], input[type="submit"]');
    
    allButtons.forEach((btn) => {
        const text = (btn.textContent || btn.value || '').toLowerCase();
        
        if (text.includes('quit') || text.includes('quitter')) {
            btn.addEventListener('click', function(e) {
                e.preventDefault();
                sendToRpi5('QUIT_TO_ZERO');
            });
        }
        
        if (text.includes('urgence') || text.includes('emergency')) {
            btn.addEventListener('click', function(e) {
                e.preventDefault();
                sendToRpi5('EMERGENCY_STOP');
            });
        }
    });
}

// =================================================================
// INITIALISATION
// =================================================================
window.addEventListener('load', function() {
    console.log('🎬 Initialisation avec débogage complet');
    
    setTimeout(() => {
        initROS();
        
        const checkConnection = setInterval(() => {
            if (isConnected) {
                clearInterval(checkConnection);
                
                setupModeControls();
                setupSystemControls();
                setupJoystickControls();
                setupWorkspaceControls();
                
                console.log('✅ Interface avec débogage prête');
            }
        }, 100);
    }, 1000);
});

// =================================================================
// FONCTIONS DE TEST ÉTENDUES
// =================================================================
window.forceSync = function() {
    console.log('🔄 Synchronisation forcée...');
    syncWorkspaceLimitsFromInterface();
};

window.debugInterface = function() {
    debugWorkspaceInterface();
};

window.showState = function() {
    console.log('📊 ÉTAT ACTUEL:');
    console.log('Position:', currentPosition);
    console.log('Limites:', currentWorkspaceLimits);
    console.log('Step size:', stepSize);
};

window.testMovement = function(axis, direction) {
    console.log(`🧪 Test mouvement ${axis} ${direction}`);
    if (checkMovementAllowed(axis, direction)) {
        sendToRpi5(`JOYSTICK:MODE1:${axis}:${direction}`);
    }
};

window.setTestLimits = function() {
    console.log('🧪 Définition limites de test...');
    currentWorkspaceLimits = {
        x_min: -10,
        x_max: 10,
        y_min: -10,
        y_max: 10,
        z_min: -10,
        z_max: 10
    };
    console.log('Limites définies:', currentWorkspaceLimits);
};

