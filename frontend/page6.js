// page6.js - Page Admin (Mot de passe incorrect)
// Gestion de la page d'erreur de connexion du système CoBra

// Fonction d'initialisation de la page d'erreur de connexion
function initLoginErrorPage() {
    console.log('Initialisation de la page d\'erreur de connexion');
    
    // Créer l'élément de contenu principal
    const loginErrorContent = document.createElement('div');
    loginErrorContent.id = 'loginError';
    loginErrorContent.className = 'page';
    loginErrorContent.style.display = 'none'; // Masqué par défaut
    
    // Créer le formulaire de connexion avec message d'erreur
    const loginForm = document.createElement('div');
    loginForm.className = 'login-form';
    
    // Ajouter un triangle d'avertissement et message d'erreur
    const errorHeader = document.createElement('div');
    errorHeader.style.textAlign = 'center';
    errorHeader.style.marginBottom = '20px';
    errorHeader.style.color = '#e74c3c';
    
    // Icône de triangle d'avertissement
    const warningIcon = document.createElement('div');
    warningIcon.innerHTML = '⚠️';
    warningIcon.style.fontSize = '32px';
    warningIcon.style.marginBottom = '10px';
    
    // Message d'erreur
    const errorText = document.createElement('div');
    errorText.textContent = 'L\'identifiant ou le mot de passe est incorrect.';
    errorText.style.fontSize = '18px';
    
    errorHeader.appendChild(warningIcon);
    errorHeader.appendChild(errorText);
    
    // Groupe d'identifiant
    const usernameGroup = document.createElement('div');
    usernameGroup.className = 'form-group';
    
    const usernameLabel = document.createElement('label');
    usernameLabel.className = 'form-label';
    usernameLabel.htmlFor = 'usernameError';
    usernameLabel.textContent = 'Identifiant :';
    
    const usernameInput = document.createElement('input');
    usernameInput.className = 'form-input';
    usernameInput.type = 'text';
    usernameInput.id = 'usernameError';
    usernameInput.placeholder = 'Entrez votre identifiant';
    usernameInput.value = 'CoBra2025'; // Valeur précédemment entrée
    
    usernameGroup.appendChild(usernameLabel);
    usernameGroup.appendChild(usernameInput);
    
    // Groupe de mot de passe
    const passwordGroup = document.createElement('div');
    passwordGroup.className = 'form-group';
    
    const passwordLabel = document.createElement('label');
    passwordLabel.className = 'form-label';
    passwordLabel.htmlFor = 'passwordError';
    passwordLabel.textContent = 'Mots de passe :';
    
    const passwordInput = document.createElement('input');
    passwordInput.className = 'form-input';
    passwordInput.type = 'password';
    passwordInput.id = 'passwordError';
    passwordInput.placeholder = 'Entrez votre mot de passe';
    passwordInput.value = 'CRIStaL'; // Valeur incorrecte (sans @)
    passwordInput.style.borderColor = '#e74c3c'; // Mise en évidence de l'erreur
    
    passwordGroup.appendChild(passwordLabel);
    passwordGroup.appendChild(passwordInput);
    
    // Actions du formulaire
    const formActions = document.createElement('div');
    formActions.className = 'form-actions';
    
    // Bouton Annuler
    const cancelButton = document.createElement('button');
    cancelButton.className = 'btn';
    cancelButton.id = 'cancelErrorButton';
    cancelButton.textContent = 'Annuler';
    
    // Bouton Connexion
    const loginButton = document.createElement('button');
    loginButton.className = 'btn';
    loginButton.id = 'loginErrorButton';
    loginButton.style.backgroundColor = '#2e6da4';
    loginButton.style.color = 'white';
    loginButton.textContent = 'Connexion';
    
    formActions.appendChild(cancelButton);
    formActions.appendChild(loginButton);
    
    // Assembler le formulaire
    loginForm.appendChild(errorHeader);
    loginForm.appendChild(usernameGroup);
    loginForm.appendChild(passwordGroup);
    loginForm.appendChild(formActions);
    
    // Ajouter le formulaire à la page
    loginErrorContent.appendChild(loginForm);
    
    // Ajouter la page au conteneur principal
    document.querySelector('.content').appendChild(loginErrorContent);
    
    // Enregistrer les gestionnaires d'événements spécifiques à cette page
    setupLoginErrorEventListeners();
    
    return loginErrorContent;
}

// Configuration des écouteurs d'événements pour la page d'erreur de connexion
function setupLoginErrorEventListeners() {
    console.log('Configuration des écouteurs d\'événements pour la page d\'erreur de connexion');
    
    // Gestionnaire pour le bouton de connexion (nouvel essai)
    const loginButton = document.getElementById('loginErrorButton');
    if (loginButton) {
        loginButton.addEventListener('click', function() {
            handleErrorLoginSubmit();
        });
    }
    
    // Gestionnaire pour le bouton d'annulation
    const cancelButton = document.getElementById('cancelErrorButton');
    if (cancelButton) {
        cancelButton.addEventListener('click', function() {
            console.log('Annulation de la connexion (page d\'erreur)');
            
            // Rediriger vers la page d'accueil
            showPage('home');
        });
    }
    
    // Gestionnaire pour la soumission du formulaire avec la touche Entrée
    const passwordInput = document.getElementById('passwordError');
    if (passwordInput) {
        passwordInput.addEventListener('keypress', function(event) {
            if (event.key === 'Enter') {
                event.preventDefault();
                handleErrorLoginSubmit();
            }
        });
    }
}

// Fonction pour gérer la soumission du formulaire de connexion depuis la page d'erreur
function handleErrorLoginSubmit() {
    console.log('Nouvelle tentative de connexion après erreur');
    
    // Récupérer les valeurs des champs
    const username = document.getElementById('usernameError').value;
    const password = document.getElementById('passwordError').value;
    
    // Vérifier les identifiants
    if (username === 'CoBra2025' && password === 'CRISt@L') {
        console.log('Connexion réussie après correction');
        
        // Masquer la bannière d'erreur globale
        document.getElementById('errorBanner').style.display = 'none';
        
        // Rediriger vers la page de calibration (Mode Admin)
        showPage('calibration');
    } else {
        console.log('Nouvel échec de connexion');
        
        // Afficher la bannière d'erreur globale
        document.getElementById('errorBanner').style.display = 'block';
        
        // Vider le champ de mot de passe
        document.getElementById('passwordError').value = '';
        
        // Mettre en évidence le champ de mot de passe
        const passwordInput = document.getElementById('passwordError');
        passwordInput.style.borderColor = '#e74c3c';
        passwordInput.focus();
    }
}

// Fonction pour nettoyer la page lors du changement de page
function cleanupLoginErrorPage() {
    console.log('Nettoyage de la page d\'erreur de connexion');
    
    // Supprimer les écouteurs d'événements pour éviter les fuites de mémoire
    const loginButton = document.getElementById('loginErrorButton');
    if (loginButton) {
        loginButton.removeEventListener('click', null);
    }
    
    const cancelButton = document.getElementById('cancelErrorButton');
    if (cancelButton) {
        cancelButton.removeEventListener('click', null);
    }
    
    const passwordInput = document.getElementById('passwordError');
    if (passwordInput) {
        passwordInput.removeEventListener('keypress', null);
    }
}

// Exportation des fonctions pour utilisation dans le module principal
export {
    initLoginErrorPage,
    handleErrorLoginSubmit,
    cleanupLoginErrorPage
};
