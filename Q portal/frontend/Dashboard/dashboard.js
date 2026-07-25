const poviForm = document.getElementById("povi-form");
const studentMessage = document.getElementById("student-message");
const poviResponse = document.getElementById("povi-response");
const signOutButton = document.getElementById("sign-out-button");
const languageSelect = document.getElementById("language-select");


//sigh out jumping to login page
signOutButton.addEventListener("click", function (event) {
    // Remove login information if you stored it
    localStorage.removeItem("loggedIn");

    // Go to the login page
    window.location.href = "login.html";
});


//language choice
const translations = {

    en: {
        title: "Share How You're Feeling",
        description: "Tell Povi what is on your mind.",
        button: "Talk to Povi"
    },

    fr: {
        title: "Partagez ce que vous ressentez",
        description: "Dites à Povi ce qui vous préoccupe.",
        button: "Parler avec Povi"
    }

};

languageSelect.addEventListener("change", function () {

    const language = this.value;

    document.getElementById("title").textContent =
        translations[language].title;

    document.getElementById("description").textContent =
        translations[language].description;

    document.getElementById("submit-button").textContent =
        translations[language].button;

});

poviForm.addEventListener("submit", function (event) {
    event.preventDefault();

    const message = studentMessage.value.trim();

    if (!message) {
        return;
    }

    poviResponse.innerHTML = `
        <div class="support-icon">♡</div>

        <div>
            <h3>Thank you for sharing</h3>

            <p>
                It sounds like you may be carrying a lot right now.
                Try choosing one manageable next step and giving
                yourself permission to take a short break.
            </p>
        </div>
    `;
});