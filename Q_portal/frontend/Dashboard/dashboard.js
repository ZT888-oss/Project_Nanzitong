const poviForm = document.getElementById("povi-form");
const studentMessage = document.getElementById("student-message");
const poviResponse = document.getElementById("povi-response");
const signOutButton = document.getElementById("sign-out-button");
const languageSelect = document.getElementById("language-select");

// mood-check modal
const modal = document.getElementById("welcome-modal");
const closeButton = document.getElementById("close-modal");
const continueButton = document.getElementById("continue-button");

// Mood selection
const moodOptions = document.querySelectorAll(".mood-option");
const moodError = document.getElementById("mood-error");
let selectedMoodLevel = null;

//recommendation 
const recommendationPanel = document.getElementById("wellbeing-recommendation");
const recommendationTitle = document.getElementById("recommendation-title");
const recommendationMessage = document.getElementById("recommendation-message");
const talkToPoviButton = document.getElementById("talk-to-povi-button");
const universitySupportLink = document.getElementById("university-support-link");
const bookSupportLink = document.getElementById("book-support-link");
const urgentSupportLink = document.getElementById("urgent-support-link");


async function loadWellbeingIndex() {
    const scoreElement = document.getElementById("wellbeing-score");
    const statusElement = document.getElementById("wellbeing-status");
    const progressBar = document.getElementById(
        "wellbeing-progress-bar"
    );
    const descriptionElement = document.getElementById(
        "wellbeing-description"
    );

    try {
        const response = await fetch("/api/wellbeing-index", {
            credentials: "include"
        });

        const data = await response.json();

        if (!response.ok) {
            throw new Error(
                data.message || "Unable to load well-being index."
            );
        }

        if (data.index === null) {
            scoreElement.textContent = "--";
            statusElement.textContent = "No data";
            progressBar.style.width = "0%";

            descriptionElement.textContent =
                "Complete your first mood check-in to generate an index.";

            return;
        }

        scoreElement.textContent = data.index;
        statusElement.textContent = data.status;
        progressBar.style.width = `${data.index}%`;


        const recommendation = data.recommendation;

        if (recommendation && recommendationPanel) {
            recommendationPanel.hidden = false;
            recommendationPanel.dataset.level = recommendation.level;

            recommendationTitle.textContent =
                recommendation.title;

            recommendationMessage.textContent =
                recommendation.message;

            talkToPoviButton.hidden =
                recommendation.primaryAction !== "talk-to-povi";

            universitySupportLink.hidden =
                recommendation.secondaryAction !==
                "view-university-support";

            bookSupportLink.hidden =
                recommendation.primaryAction !== "book-support";

            urgentSupportLink.hidden =
                recommendation.secondaryAction !==
                "view-urgent-resources";
        }

        if (talkToPoviButton) {
            talkToPoviButton.addEventListener("click", function () {
                studentMessage.focus();

                poviForm.scrollIntoView({
                    behavior: "smooth",
                    block: "center"
                });
            });
        }

    } catch (error) {
        console.error(error);

        statusElement.textContent = "Unavailable";
        descriptionElement.textContent = error.message;
    }
}


async function showMoodModalWhenNeeded() {
    try {
        const response = await fetch("/api/mood-checkins", {
            credentials: "include"
        });

        const contentType =
            response.headers.get("content-type");

        if (
            !contentType ||
            !contentType.includes("application/json")
        ) {
            const text = await response.text();

            throw new Error(
                `Expected JSON but received: ${text.slice(0, 80)}`
            );
        }

        const data = await response.json();

        if (!response.ok) {
            throw new Error(
                data.message ||
                "Unable to check today's mood status."
            );
        }

        if (!data.checkedInToday && modal) {
            modal.classList.add("show");
        } else if (modal) {
            modal.classList.remove("show");
        }
    } catch (error) {
        console.error(
            "Check daily mood status error:",
            error
        );
    }
}


document.addEventListener("DOMContentLoaded", function () {

    // Sign out
    if (signOutButton) {
        signOutButton.addEventListener("click", function () {
            localStorage.removeItem("loggedIn");
            window.location.href = "login.html";
        });
    }
    //decide if show pop out check-in window
    showMoodModalWhenNeeded();

    // const shouldShowPopup =
    //     sessionStorage.getItem("showLoginPopup") === "true";

    // if (modal && shouldShowPopup) {
    //     modal.classList.add("show");
    //     sessionStorage.removeItem("showLoginPopup");
    // }

    function closeModal() {
        if (modal) {
            modal.classList.remove("show");
        }
    }

    if (closeButton) {
        closeButton.addEventListener("click", closeModal);
    }

    if (modal) {
        modal.addEventListener("click", function (event) {
            if (event.target === modal) {
                closeModal();
            }
        });
    }


    continueButton.addEventListener("click", async function () {
        if (selectedMoodLevel === null) {
            moodError.textContent = "Please select your current mood.";
            return;
        }


        //sned the mood to backend to evaluate well-being Index
        try {
            const response = await fetch("/api/mood-checkins", {
                method: "POST",
                headers: {
                    "Content-Type": "application/json"
                },

                credentials: "include",

                body: JSON.stringify({
                    moodLevel: selectedMoodLevel
                })
            });

            const data = await response.json();

            if (!response.ok) {
                throw new Error(data.message || "Unable to save mood.");
            }

            console.log("Mood saved:", data);

            closeModal();

            // Refresh the dashboard card
            await loadWellbeingIndex();
        } catch (error) {
            moodError.textContent = error.message;
            console.error(error);
        }
    });


    //mood check-in
    moodOptions.forEach(function (option) {
        option.addEventListener("click", function () {
            moodOptions.forEach(function (mood) {
                mood.classList.remove("selected");
            });

            option.classList.add("selected");

            selectedMoodLevel = Number(option.dataset.level);

            moodError.textContent = "";

            console.log("Selected mood level:", selectedMoodLevel);
        });
    });

    //load mood index
    loadWellbeingIndex();


    //========================
    // Language selection
    //==========================

    const translations = {
        English: {
            title: "Share How You're Feeling",
            description: "Tell Povi what is on your mind.",
            button: "Talk to Povi"
        },

        French: {
            title: "Partagez ce que vous ressentez",
            description: "Dites à Povi ce qui vous préoccupe.",
            button: "Parler avec Povi"
        }
    };

    if (languageSelect) {
        languageSelect.addEventListener("change", function () {
            const language = this.value;
            const selectedTranslation = translations[language];

            if (!selectedTranslation) {
                return;
            }

            document.getElementById("title").textContent =
                selectedTranslation.title;

            document.getElementById("description").textContent =
                selectedTranslation.description;

            document.getElementById("submit-button").textContent =
                selectedTranslation.button;
        });
    }

    // Povi form
    if (poviForm) {
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
    }
});