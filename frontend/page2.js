// page2.js - Page À propos
// Gestion de la page "À propos" du système CoBra

// Fonction d'initialisation de la page À propos
function initAboutPage() {
    console.log('Initialisation de la page À propos');
    
    // Créer l'élément de contenu principal
    const aboutContent = document.createElement('div');
    aboutContent.id = 'about';
    aboutContent.className = 'page';
    aboutContent.style.display = 'none'; // Masqué par défaut
    
    // Créer le titre de la page
    const pageTitle = document.createElement('h2');
    pageTitle.className = 'page-title';
    pageTitle.textContent = 'À Propos du Projet CoBra';
    
    // Créer le conteneur de texte
    const textContent = document.createElement('div');
    textContent.className = 'text-content';
    
    // Contenu textuel
    const paragraphs = [
        'Le projet COBRA consiste à développer une réplique pédagogique d\'un robot médical pour la formation à la curiethérapie.',
        'Il permet de simuler des gestes médicaux avec précision, sans contraintes IRM, à l\'aide d\'une interface intuitive et d\'une manette de contrôle.',
        'L\'objectif est de proposer une solution transportable et fiable pour les démonstrations et l\'enseignement médical.'
    ];
    
    // Ajouter les paragraphes
    paragraphs.forEach(text => {
        const paragraph = document.createElement('p');
        paragraph.textContent = text;
        textContent.appendChild(paragraph);
    });
    
    // Assembler les éléments
    aboutContent.appendChild(pageTitle);
    aboutContent.appendChild(textContent);
    
    // Ajouter la page au conteneur principal
    document.querySelector('.content').appendChild(aboutContent);
    
    // Enregistrer les gestionnaires d'événements spécifiques à cette page
    setupAboutEventListeners();
    
    return aboutContent;
}

// Configuration des écouteurs d'événements pour la page À propos
function setupAboutEventListeners() {
    console.log('Configuration des écouteurs d\'événements pour la page À propos');
    
    // Exemple : effet de survol sur les paragraphes
    const paragraphs = document.querySelectorAll('#about p');
    paragraphs.forEach(p => {
        p.addEventListener('mouseenter', function() {
            this.style.color = '#2e6da4';
            this.style.transition = 'color 0.3s ease';
        });
        
        p.addEventListener('mouseleave', function() {
            this.style.color = '';
        });
    });
}

// Fonction pour mettre à jour le contenu de la page À propos
function updateAboutPage(data) {
    console.log('Mise à jour de la page À propos avec de nouvelles données', data);
    
    if (!data) return;
    
    // Mise à jour du contenu textuel si fourni
    if (data.paragraphs && Array.isArray(data.paragraphs)) {
        const textContent = document.querySelector('#about .text-content');
        if (textContent) {
            // Vider le contenu actuel
            textContent.innerHTML = '';
            
            // Ajouter les nouveaux paragraphes
            data.paragraphs.forEach(text => {
                const paragraph = document.createElement('p');
                paragraph.textContent = text;
                textContent.appendChild(paragraph);
            });
            
            // Réappliquer les écouteurs d'événements
            setupAboutEventListeners();
        }
    }
    
    // Mise à jour du titre si fourni
    if (data.title) {
        const title = document.querySelector('#about .page-title');
        if (title) {
            title.textContent = data.title;
        }
    }
}

// Fonction pour nettoyer la page lors du changement de page
function cleanupAboutPage() {
    console.log('Nettoyage de la page À propos');
    
    // Supprimer les écouteurs d'événements pour éviter les fuites de mémoire
    const paragraphs = document.querySelectorAll('#about p');
    paragraphs.forEach(p => {
        p.removeEventListener('mouseenter', null);
        p.removeEventListener('mouseleave', null);
    });
}

// Exportation des fonctions pour utilisation dans le module principal
export {
    initAboutPage,
    updateAboutPage,
    cleanupAboutPage
};
