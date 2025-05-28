// main.js - Code principal regroupant toutes les pages
// Initialisation et gestion globale de l'application CoBra

// Variables globales
let currentPage = 'home';
let isAdminLoggedIn = false;
const validCredentials = {
    username: 'CoBra2025',
    password: 'CRISt@L'
};

// Fonction d'initialisation au chargement de la page
document.addEventListener('DOMContentLoaded', function() {
    console.log('Application CoBra -    Système de Contrôle médical initialisée');
    
    // Afficher la page d'accueil par défaut
    showPage('home');
    
    // Initialiser les écouteurs d'événements globaux
    document.getElementById('speedSlider').addEventListener('input', updateSpeedSlider);
    
    // Initialiser les joysticks avec des événements tactiles
    initializeJoysticks();
    
    // Ajouter les écouteurs pour les boutons de connexion/annulation dans l'admin
    document.getElementById('loginButton').addEventListener('click', handleLogin);
    document.getElementById('cancelLoginButton').addEventListener('click', () => showPage('home'));
    
    // Ajouter les écouteurs pour les actions de réinitialisation et d'arrêt d'urgence
    document.querySelectorAll('.reset-btn').forEach(btn => {
        btn.addEventListener('click', resetSystem);
    });
    
    document.querySelectorAll('.emergency-btn').forEach(btn => {
        btn.addEventListener('click', emergencyStop);
    });
    
    // Initialiser les positions de l'aiguille
    updateNeedlePosition(28, -16, 5);
});

// Fonction pour afficher une page spécifique
function showPage(pageName) {
    console.log('Affichage de la page:', pageName);
    
    // Si l'utilisateur tente d'accéder à la page admin en mode calibration sans être connecté
    if (pageName === 'calibration' && !isAdminLoggedIn) {
        console.log('Tentative d\'accès à la page de calibration sans authentification');
        showPage('admin');
        return;
    }
    
    // Masquer toutes les pages
    document.querySelectorAll('.page').forEach(page => {
        page.style.display = 'none';
    });
    
    // Réinitialiser les boutons actifs
    document.querySelectorAll('.sidebar .btn').forEach(btn => {
        btn.classList.remove('active');
    });
    
    // Afficher la page demandée
    if (pageName === 'home') {
        document.getElementById('home').style.display = 'block';
    } else {
        document.getElementById(pageName).style.display = 'block';
        
        // Activer le bouton correspondant dans la barre latérale
        const btnSelector = '.sidebar .btn:nth-child(' + 
            (['about', 'mode1', 'mode2'].indexOf(pageName) + 1) + ')';
        const activeBtn = document.querySelector(btnSelector);
        if (activeBtn) activeBtn.classList.add('active');
        
        if (pageName === 'admin') {
            document.querySelector('.sidebar .btn:last-child').classList.add('active');
        }
    }
    
    // Masquer la bannière d'erreur si elle est visible
    document.getElementById('errorBanner').style.display = 'none';
    
    // Mettre à jour la page courante
    currentPage = pageName;
}

// Gestionnaire de connexion admin
function handleLogin() {
    const usernameInput = document.getElementById('username').value;
    const passwordInput = document.getElementById('password').value;
    
    if (usernameInput === validCredentials.username && 
        passwordInput === validCredentials.password) {
        console.log('Connexion réussie en tant qu\'administrateur');
        isAdminLoggedIn = true;
        showPage('calibration');
    } else {
        console.log('Échec de connexion: identifiants incorrects');
        document.getElementById('errorBanner').style.display = 'block';
    }
}

// Mise à jour du slider de vitesse
function updateSpeedSlider() {
    const slider = document.getElementById('speedSlider');
    const value = slider.value;
    
    // Calculer le pourcentage de progression
    const percentage = value;
    
    // Mettre à jour l'arrière-plan du slider avec un dégradé
    slider.style.background = `linear-gradient(to right, #4caf50 ${percentage}%, #e0e0e0 ${percentage}%)`;
    
    console.log('Vitesse mise à jour:', value);
}

// Fonction pour initialiser les joysticks
function initializeJoysticks() {
    // Implémentation simplifiée - dans une application réelle,
    // cette fonction gérerait les événements tactiles et les mouvements des joysticks
    console.log('Joysticks initialisés');
}

// Fonction pour mettre à jour la position de l'aiguille
function updateNeedlePosition(x, y, z) {
    // Mettre à jour les coordonnées affichées
    document.querySelectorAll('.coordinates .coordinate').forEach((coord, index) => {
        const value = [x, y, z][index];
        const span = coord.querySelector('span');
        
        // Mettre à jour la valeur
        span.textContent = (value >= 0 ? '+' : '') + value;
        
        // Mettre à jour la classe pour la couleur
        span.className = value >= 0 ? 'positive' : 'negative';
    });
    
    console.log('Position de l\'aiguille mise à jour:', { x, y, z });
}

// Fonction pour réinitialiser le système
function resetSystem() {
    console.log('Réinitialisation du système');
    
    // Réinitialiser la position de l'aiguille à zéro
    updateNeedlePosition(0, 0, 0);
    
    // Réinitialiser le slider de vitesse
    document.getElementById('speedSlider').value = 50;
    updateSpeedSlider();
    
    // Afficher une notification à l'utilisateur
    alert('Le système a été réinitialisé avec succès.');
}

// Fonction pour l'arrêt d'urgence
function emergencyStop() {
    console.log('ARRÊT D\'URGENCE ACTIVÉ');
    
    // En cas réel, cette fonction arrêterait immédiatement tous les mouvements
    // et mettrait le système en état sécurisé
    
    // Notification à l'utilisateur
    alert('ARRÊT D\'URGENCE : Tous les mouvements ont été arrêtés.');
}

// Fonction pour quitter l'application
function quitApp() {
    if (confirm('Êtes-vous sûr de vouloir quitter et revenir à l\'accueil?')) {
        console.log('Redirection vers la page d\'accueil');
        // Dans un environnement web, rediriger vers une page de déconnexion
        //window.location.href = 'logout.html';
        // Rediriger vers la page d'accueil au lieu de fermer l'application
        showPage('home');
        // Pour cette démo, on simule une fermeture
        //document.body.innerHTML = '<div style="text-align:center; margin-top:100px;"><h1>Application fermée</h1><p>L\'application CoBra a été fermée avec succès.</p><button onclick="window.location.reload()">Redémarrer</button></div>';
    }
}
// Fonction pour "quitter" l'application (rediriger vers la page d'accueil)
//export function quitApp() {
    //console.log('Redirection vers la page d\'accueil');
    // Rediriger vers la page d'accueil sans demande de confirmation
    //showPage('home');
//}

// Exportation des fonctions pour utilisation dans d'autres modules
export {
    showPage,
    handleLogin,
    updateSpeedSlider,
    updateNeedlePosition,
    resetSystem,
    emergencyStop,
    quitApp
};