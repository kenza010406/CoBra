// script.js - Fichier d'intégration principal
// Ce fichier importe tous les modules et initialise l'application

// Importer les modules
import * as MainModule from './main.js';
import { initHomePage } from './page1.js';
import { initAboutPage } from './page2.js';
import { initMode1Page } from './page3.js';
import { initMode2Page } from './page4.js';
import { initAdminLoginPage } from './page5.js';
import { initLoginErrorPage } from './page6.js';
import { initCalibrationPage } from './page7.js';

// Fonction d'initialisation de l'application
function initApp() {
    console.log('Initialisation de l\'application CoBra');
    
    // Initialiser toutes les pages
    initHomePage();
    initAboutPage();
    initMode1Page();
    initMode2Page();
    initAdminLoginPage();
    initLoginErrorPage();
    initCalibrationPage();
    
    // Rendre les fonctions globales accessibles au HTML
    window.showPage = MainModule.showPage;
    window.quitApp = MainModule.quitApp;
    
    // Initialiser les écouteurs d'événements globaux
    initializeGlobalEventListeners();
    
    // Afficher la page d'accueil par défaut
    MainModule.showPage('home');
}

// Fonction pour initialiser les écouteurs d'événements globaux
function initializeGlobalEventListeners() {
    // Initialiser les curseurs de vitesse
    document.querySelectorAll('.slider').forEach(slider => {
        slider.addEventListener('input', function() {
            // Calculer le pourcentage de progression
            const percentage = (this.value - this.min) / (this.max - this.min) * 100;
            
            // Mettre à jour l'arrière-plan du curseur avec un dégradé
            this.style.background = `linear-gradient(to right, #4caf50 ${percentage}%, #e0e0e0 ${percentage}%)`;
        });
        
        // Déclencher l'événement input pour initialiser l'arrière-plan
        const event = new Event('input');
        slider.dispatchEvent(event);
    });
}

// Initialiser l'application au chargement du document
document.addEventListener('DOMContentLoaded', initApp);
