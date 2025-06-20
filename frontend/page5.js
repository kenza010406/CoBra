// page5.js - Page Admin (Login)
// Gestion de la page de connexion administrateur du système CoBra

// Fonction d'initialisation de la page de connexion admin
function initAdminLoginPage() {
    console.log('Initialisation de la page de connexion admin');
    
    // Créer l'élément de contenu principal
    const adminContent = document.createElement('div');
    adminContent.id = 'admin';
    adminContent.className = 'page';
    adminContent.style.display = 'none'; // Masqué par défaut
    
    // Créer le formulaire de connexion
    const loginForm = document.createElement('div');
    loginForm.className = 'login-form';
    
    // Message d'erreur (masqué par défaut)
    const errorMessage = document.createElement('div');
    errorMessage.className = 'error-message';
    errorMessage.id = 'loginErrorMessage';
    errorMessage.style.display = 'none';
    
    // Icône d'erreur et texte
    const errorIcon = document.createElement('span');
    errorIcon.className = 'error-icon';
    errorIcon.textContent = '⚠️';
    
    const errorText = document.createElement('span');
    errorText.textContent = 'L\'identifiant ou le mot de passe est incorrect.';
    
    errorMessage.appendChild(errorIcon);
    errorMessage.appendChild(errorText);
    
    // Groupe d'identifiant
    const usernameGroup = document.createElement('div');
    usernameGroup.className = 'form-group';
    
    const usernameLabel = document.createElement('label');
    usernameLabel.className = 'form-label';
    usernameLabel.htmlFor = 'username';
    usernameLabel.textContent = 'Identifiant :';
    
    const usernameInput = document.createElement('input');
    usernameInput.className = 'form-input';
    usernameInput.type = 'text';
    usernameInput.id = 'username';
    usernameInput.placeholder = 'Entrez votre identifiant';
    
    usernameGroup.appendChild(usernameLabel);
    usernameGroup.appendChild(usernameInput);
    
    // Groupe de mot de passe
    const passwordGroup = document.createElement('div');
    passwordGroup.className = 'form-group';
    
    const passwordLabel = document.createElement('label');
    passwordLabel.className = 'form-label';
    passwordLabel.htmlFor = 'password';
    passwordLabel.textContent = 'Mots de passe :';
    
    const passwordInput = document.createElement('input');
    passwordInput.className = 'form-input';
    passwordInput.type = 'password';
    passwordInput.id = 'password';
    passwordInput.placeholder = 'Entrez votre mot de passe';
    
    passwordGroup.appendChild(passwordLabel);
    passwordGroup.appendChild(passwordInput);
    
    // Actions du formulaire
    const formActions = document.createElement('div');
    formActions.className = 'form-actions';
    
    // Bouton Annuler
    const cancelButton = document.createElement('button');
    cancelButton.className = 'btn';
    cancelButton.id = 'cancelLoginButton';
    cancelButton.textContent = 'Annuler';
    
    // Bouton Connexion
    const loginButton = document.createElement('button');
    loginButton.className = 'btn';
    loginButton.id = 'loginButton';
    loginButton.style.backgroundColor = '#2e6da4';
    loginButton.style.color = 'white';
    loginButton.textContent = 'Connexion';
    
    formActions.appendChild(cancelButton);
    formActions.appendChild(loginButton);
    
    // Assembler le formulaire
    loginForm.appendChild(errorMessage);
    loginForm.appendChild(usernameGroup);
    loginForm.appendChild(passwordGroup);
    loginForm.appendChild(formActions);
    
    // Ajouter le formulaire à la page
    adminContent.appendChild(loginForm);
    
    // Ajouter la page au conteneur principal
    document.querySelector('.content').appendChild(adminContent);
    
    // Enregistrer les gestionnaires d'événements spécifiques à cette page
    setupAdminLoginEventListeners();
    
    return adminContent;
}

// Configuration des écouteurs d'événements pour la page de connexion
function setupAdminLoginEventListeners() {
    console.log('Configuration des écouteurs d\'événements pour la page de connexion admin');
    
    // Gestionnaire pour le bouton de connexion
    const loginButton = document.getElementById('loginButton');
    if (loginButton) {
        loginButton.addEventListener('click', function() {
            handleLoginSubmit();
        });
    }
    
    // Gestionnaire pour le bouton d'annulation
    const cancelButton = document.getElementById('cancelLoginButton');
    if (cancelButton) {
        cancelButton.addEventListener('click', function() {
            console.log('Annulation de la connexion');
            
            // Réinitialiser le formulaire
            resetLoginForm();
            
            // Rediriger vers la page d'accueil
            showPage('home');
        });
    }
    
    // Gestionnaire pour la soumission du formulaire avec la touche Entrée
    const passwordInput = document.getElementById('password');
    if (passwordInput) {
        passwordInput.addEventListener('keypress', function(event) {
            if (event.key === 'Enter') {
                event.preventDefault();
                handleLoginSubmit();
            }
        });
    }
}

// Fonction pour gérer la soumission du formulaire de connexion
function handleLoginSubmit() {
    console.log('Tentative de connexion');
    
    // Récupérer les valeurs des champs
    const username = document.getElementById('username').value;
    const password = document.getElementById('password').value;
    
    // Vérifier les identifiants (exemple simple)
    // Dans une application réelle, cela serait fait côté serveur
    if (username === 'CoBra2025' && password === 'CRISt@L') {
        console.log('Connexion réussie');
        
        // Réinitialiser le message d'erreur s'il est affiché
        const errorMessage = document.getElementById('loginErrorMessage');
        if (errorMessage) {
            errorMessage.style.display = 'none';
        }
        
        // Masquer la bannière d'erreur globale
        document.getElementById('errorBanner').style.display = 'none';
        
        // Réinitialiser le formulaire
        resetLoginForm();
        
        // Rediriger vers la page de calibration (Mode Admin)
        showPage('calibration');
    } else {
        console.log('Échec de connexion: identifiants incorrects');
        
        // Afficher le message d'erreur local
        const errorMessage = document.getElementById('loginErrorMessage');
        if (errorMessage) {
            errorMessage.style.display = 'block';
        }
        
        // Afficher la bannière d'erreur globale
        document.getElementById('errorBanner').style.display = 'block';
        
        // Vider le champ de mot de passe
        document.getElementById('password').value = '';
    }
}

// Fonction pour réinitialiser le formulaire de connexion
function resetLoginForm() {
    document.getElementById('username').value = '';
    document.getElementById('password').value = '';
    
    // Masquer le message d'erreur s'il est affiché
    const errorMessage = document.getElementById('loginErrorMessage');
    if (errorMessage) {
        errorMessage.style.display = 'none';
    }
}

// Fonction pour mettre à jour la page Admin (login)
function updateAdminLoginPage(data) {
    console.log('Mise à jour de la page Admin (login) avec de nouvelles données', data);
    
    if (!data) return;
    
    // Exemple: mise à jour des champs de saisie si fournis
    if (data.username) {
        document.getElementById('username').value = data.username;
    }
}

// Fonction pour nettoyer la page lors du changement de page
function cleanupAdminLoginPage() {
    console.log('Nettoyage de la page Admin (login)');
    
    // Réinitialiser le formulaire
    resetLoginForm();
    
    // Supprimer les écouteurs d'événements pour éviter les fuites de mémoire
    const loginButton = document.getElementById('loginButton');
    if (loginButton) {
        loginButton.removeEventListener('click', null);
    }
    
    const cancelButton = document.getElementById('cancelLoginButton');
    if (cancelButton) {
        cancelButton.removeEventListener('click', null);
    }
    
    const passwordInput = document.getElementById('password');
    if (passwordInput) {
        passwordInput.removeEventListener('keypress', null);
    }
}

// Exportation des fonctions pour utilisation dans le module principal
export {
    initAdminLoginPage,
    handleLoginSubmit,
    resetLoginForm,
    updateAdminLoginPage,
    cleanupAdminLoginPage
};
