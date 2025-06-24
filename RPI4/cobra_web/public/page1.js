// page1.js - Page d'accueil
// Gestion de la page d'accueil du système CoBra

// Fonction d'initialisation de la page d'accueil
function initHomePage() {
    console.log('Initialisation de la page d\'accueil');
    
    // Créer l'élément de contenu principal
    const homeContent = document.createElement('div');
    homeContent.id = 'home';
    homeContent.className = 'page';
    
    // Conteneur principal avec flexbox pour centrer verticalement
    const mainContainer = document.createElement('div');
    mainContainer.style.display = 'flex';
    mainContainer.style.flexDirection = 'column';
    mainContainer.style.justifyContent = 'center'; // Centre verticalement
    mainContainer.style.alignItems = 'center'; // Centre horizontalement
    mainContainer.style.height = '100%'; // Utiliser 100% de la hauteur disponible
    mainContainer.style.padding = '20px'; // Marge uniforme
    
    // Créer un conteneur circulaire pour l'image
    const circleContainer = document.createElement('div');
    circleContainer.style.width = '400px'; // Taille légèrement réduite
    circleContainer.style.height = '400px'; // Garder forme circulaire avec dimensions égales
    circleContainer.style.borderRadius = '50%'; // Forme circulaire
    circleContainer.style.overflow = 'hidden'; // Masquer ce qui dépasse
    circleContainer.style.position = 'relative'; // Pour positionnement absolu de l'image
    circleContainer.style.boxShadow = '0 5px 20px rgba(0,0,0,0.15)'; // Ombre plus prononcée
    circleContainer.style.margin = '0 auto 30px auto'; // Marge en bas réduite
    
    // Ajouter l'image du robot médical
    const robotImage = document.createElement('img');
    robotImage.src = 'robot-medical.png';
    robotImage.alt = 'Robot médical';
    
    // Positionner l'image pour montrer la partie pertinente (le bras robotique)
    robotImage.style.position = 'absolute';
    robotImage.style.width = '150%'; // Image plus grande que le conteneur
    robotImage.style.height = 'auto'; // Conserver les proportions
    robotImage.style.top = '-15%'; // Ajuster verticalement
    robotImage.style.left = '-25%'; // Ajuster horizontalement
    robotImage.style.transform = 'scale(1)'; // État initial pour l'animation
    robotImage.style.transition = 'transform 0.5s ease'; // Animation fluide
    
    // Ajouter l'image au conteneur
    circleContainer.appendChild(robotImage);
    
    // Créer le texte descriptif
    const description = document.createElement('h3');
    description.textContent = 'Système de positionnement précis d\'aiguille pour intervention médicales';
    description.style.textAlign = 'center'; // Centrer le texte
    description.style.color = '#914696'; // Couleur bleue assortie
    description.style.fontSize = '24px'; // Taille de police plus grande
    description.style.fontWeight = '500'; // Légèrement moins gras qu'un h3 normal
    description.style.margin = '0'; // Supprimer marges par défaut
    description.style.maxWidth = '90%'; // Limiter la largeur
    description.style.lineHeight = '1.4'; // Espacement des lignes amélioré
    
    // Assembler les éléments
    mainContainer.appendChild(circleContainer);
    mainContainer.appendChild(description);
    homeContent.appendChild(mainContainer);
    
    // Ajouter la page au conteneur principal
    document.querySelector('.content').appendChild(homeContent);
    
    // Enregistrer les gestionnaires d'événements
    setupHomeEventListeners();
    
    return homeContent;
}

// Configuration des écouteurs d'événements pour la page d'accueil
function setupHomeEventListeners() {
    console.log('Configuration des écouteurs d\'événements pour la page d\'accueil');
    
    // Animation sur l'image au survol
    const robotImage = document.querySelector('#home img');
    if (robotImage) {
        robotImage.addEventListener('mouseenter', function() {
            this.style.transform = 'scale(1.05)'; // Légère augmentation de taille
            // Pas de rotation pour garder l'image professionnelle
        });
        
        robotImage.addEventListener('mouseleave', function() {
            this.style.transform = 'scale(1)'; // Retour à la taille normale
        });
    }
}

// Fonction pour mettre à jour le contenu de la page d'accueil
function updateHomePage(data) {
    console.log('Mise à jour de la page d\'accueil avec de nouvelles données', data);
    
    if (!data) return;
    
    // Mise à jour du texte de description si fourni
    if (data.description) {
        const description = document.querySelector('#home h3');
        if (description) {
            description.textContent = data.description;
        }
    }
    
    // Mise à jour de l'image si fournie
    if (data.imageSrc) {
        const image = document.querySelector('#home img');
        if (image) {
            image.src = data.imageSrc;
        }
    }
}

// Fonction pour nettoyer la page lors du changement de page
function cleanupHomePage() {
    console.log('Nettoyage de la page d\'accueil');
    
    // Supprimer les écouteurs d'événements pour éviter les fuites de mémoire
    const robotImage = document.querySelector('#home img');
    if (robotImage) {
        robotImage.removeEventListener('mouseenter', null);
        robotImage.removeEventListener('mouseleave', null);
    }
}

// Exportation des fonctions pour utilisation dans le module principal
export {
    initHomePage,
    updateHomePage,
    cleanupHomePage
};
const style = document.createElement('style');
style.textContent = `
    /* Centrage de tout le contenu de la page accueil */
    #home.page {
        display: flex;
        flex-direction: column;
        align-items: center;
        justify-content: center;
        height: 50vh;
        text-align: center;
    }

    .page-title {
        font-size: 40px;
        margin-bottom: 20px;
    }

    .robot-container {
        display: flex;
        justify-content: center;
        margin-bottom: 30px;
    }

    .robot-image {
        max-width: 600px;
        width: 100%;
        height: auto;
    }

    .description h3 {
        font-size: 18px;
        margin: 0;
    }
`;
document.head.appendChild(style);
