const express = require("express"); //webserver
const Database = require("better-sqlite3"); //database
const bcrypt = require("bcrypt"); //password_hash
const session = require("express-session");
const path = require("path"); //file path
const app = express(); //save Express application into app
const PORT = 3000;

const db = new Database("users.db");
db.pragma("foreign_keys = ON");

//create user_info table only do when it not exist
db.prepare(`
    CREATE TABLE IF NOT EXISTS user_info (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        username TEXT NOT NULL,
        email TEXT NOT NULL UNIQUE,
        password_hash TEXT NOT NULL,
        preferred_language TEXT NOT NULL DEFAULT 'en',
        created_at DATETIME DEFAULT CURRENT_TIMESTAMP
    )
`).run();


db.prepare(`
    CREATE TABLE IF NOT EXISTS mood_checkins (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        user_id INTEGER NOT NULL,
        mood_level INTEGER NOT NULL CHECK (mood_level BETWEEN 1 AND 5),
        created_at DATETIME DEFAULT CURRENT_TIMESTAMP,

        FOREIGN KEY (user_id) REFERENCES user_info(id)
    )
`).run();

//convert incoming JSON back to javascript object that express can understand
app.use(express.json());  //For incoming requests, use Express's JSON-reading functionality.


//For incoming requests, use Express's static-file-serving functionality
app.use(
    express.static(
        path.join(__dirname, "../frontend") //Serve all the files inside the frontend folder
    )
);

//defien req.seesion
app.use(session({
    secret: "q-portal-secret-key",
    resave: false,
    saveUninitialized: false,
    cookie: {
        secure: false
    }
}));


// examine register information when frontend send information to backend
app.post("/api/register", async (request, response) => {
    try {
        const {
            username,
            email,
            password,
            preferredLanguage = "en"
        } = request.body;

        if (!username || !email || !password) {
            return response.status(400).json({
                success: false,
                message: "All fields are required."
            });
        }

        const existingUser = db.prepare(`
            SELECT id
            FROM user_info
            WHERE username = ? OR email = ?
        `).get(username, email);

        if (existingUser) {
            return response.status(409).json({
                success: false,
                message: "Username or email already exists."
            });
        }

        const passwordHash = await bcrypt.hash(password, 10);

        const result = db.prepare(`
            INSERT INTO user_info (
                username,
                email,
                password_hash,
                preferred_language
            )
            VALUES (?, ?, ?, ?)
        `).run(
            username,
            email,
            passwordHash,
            preferredLanguage
        );

        response.status(201).json({
            success: true,
            userId: result.lastInsertRowid,
            message: "Account created successfully."
        });
    } catch (error) {
        console.error(error);

        response.status(500).json({
            success: false,
            message: "Server error."
        });
    }
});


//examine login information when frontend send request
app.post("/api/login", async (request, response) => {
    try {
        const { username, password } = request.body;

        if (!username || !password) {
            return response.status(400).json({
                success: false,
                message: "Username and password are required."
            });
        }

        const user = db.prepare(`
            SELECT
                id,
                username,
                email,
                password_hash,
                preferred_language
            FROM user_info
            WHERE username = ? OR email = ?
        `).get(username, username);

        if (!user) {
            return response.status(401).json({
                success: false,
                message: "Invalid username or password."
            });
        }

        const passwordMatches = await bcrypt.compare(
            password,
            user.password_hash
        );

        if (!passwordMatches) {
            return response.status(401).json({
                success: false,
                message: "Invalid username or password."
            });
        }

        request.session.userId = user.id;

        response.json({
            success: true,
            user: {
                id: user.id,
                username: user.username,
                email: user.email,
                preferredLanguage: user.preferred_language
            }
        });
    } catch (error) {
        console.error(error);

        response.status(500).json({
            success: false,
            message: "Server error."
        });
    }
});


//create today check-in
app.post("/api/mood-checkins", requireLogin, function (req, res) {
    try {
        const userId = req.session.userId;
        const moodLevel = Number(req.body.moodLevel);

        if (
            !Number.isInteger(moodLevel) ||
            moodLevel < 1 ||
            moodLevel > 5
        ) {
            return res.status(400).json({
                success: false,
                message: "Mood level must be an integer from 1 to 5."
            });
        }

        // Check whether the user already completed today's check-in
        const existingCheckin = db.prepare(`
            SELECT id
            FROM mood_checkins
            WHERE user_id = ?
              AND DATE(created_at, 'localtime')
                  = DATE('now', 'localtime')
            LIMIT 1
        `).get(userId);

        if (existingCheckin) {
            return res.status(409).json({
                success: false,
                message:
                    "You have already completed today's mood check-in."
            });
        }

        const result = db.prepare(`
            INSERT INTO mood_checkins (
                user_id,
                mood_level
            )
            VALUES (?, ?)
        `).run(userId, moodLevel);

        return res.status(201).json({
            success: true,
            message: "Mood check-in saved successfully.",
            moodId: result.lastInsertRowid,
            moodLevel
        });
    } catch (error) {
        console.error("Save mood error:", error);

        return res.status(500).json({
            success: false,
            message: "Unable to save mood check-in."
        });
    }
});


//login to get good check-in!
function requireLogin(req, res, next) {
    if (!req.session.userId) {
        return res.status(401).json({
            message: "You must log in first."
        });
    }

    next();
}


//Backend: save well-being index into databse
app.get("/api/wellbeing-index", requireLogin, function (req, res) {
    try {
        const userId = req.session.userId;

        const result = db.prepare(`
            SELECT
                AVG(mood_level) AS average_mood,
                COUNT(*) AS checkin_count
            FROM (
                SELECT mood_level
                FROM mood_checkins
                WHERE user_id = ?
                ORDER BY created_at DESC
                LIMIT 7
            )
        `).get(userId);

        if (!result || result.checkin_count === 0) {
            return res.json({
                success: true,
                index: null,
                averageMood: null,
                checkinCount: 0,
                status: "No check-ins yet",
                recommendation: getWellbeingRecommendation(null)
            });
        }

        const averageMood = Number(result.average_mood);

        const wellbeingIndex = Math.round(
            ((averageMood - 1) / 4) * 100
        );

        let status;

        if (wellbeingIndex >= 80) {
            status = "Doing well";
        } else if (wellbeingIndex >= 60) {
            status = "Generally positive";
        } else if (wellbeingIndex >= 40) {
            status = "Mixed";
        } else if (wellbeingIndex >= 20) {
            status = "Having a difficult time";
        } else {
            status = "May need additional support";
        }

        const recommendation =
            getWellbeingRecommendation(wellbeingIndex);

        return res.json({
            success: true,
            index: wellbeingIndex,
            averageMood: Number(averageMood.toFixed(2)),
            checkinCount: result.checkin_count,
            status,
            recommendation
        });

    } catch (error) {
        console.error("Well-being index error:", error);

        return res.status(500).json({
            success: false,
            message: "Unable to calculate well-being index."
        });
    }
});

//pop out window only show onece a day(according to local time not 24hour), let server check database
app.get("/api/mood-checkins", requireLogin, function (req, res) {
    try {
        const userId = req.session.userId;

        const todayCheckin = db.prepare(`
            SELECT
                id,
                mood_level,
                created_at
            FROM mood_checkins
            WHERE user_id = ?
              AND DATE(created_at, 'localtime')
                  = DATE('now', 'localtime')
            ORDER BY created_at DESC
            LIMIT 1
        `).get(userId);

        return res.json({
            success: true,
            checkedInToday: Boolean(todayCheckin),
            checkin: todayCheckin || null
        });
    } catch (error) {
        console.error("Today's check-in error:", error);

        return res.status(500).json({
            success: false,
            message: "Unable to check today's mood status."
        });
    }
});



//recommendation section: 
function getWellbeingRecommendation(score) {
    if (score === null) {
        return {
            level: "no-data",
            title: "Complete a mood check-in",
            message:
                "Complete your first check-in to receive personalized guidance.",
            primaryAction: null,
            urgent: false
        };
    }

    if (score >= 50) {
        const encouragingMessages = [
            "You appear to be doing well. Keep making time for the habits and people that support you.",
            "Your recent check-ins look positive. Continue taking care of your sleep, routines, and relationships.",
            "You seem to be maintaining positive well-being. Remember that support is always available when needed."
        ];

        const message =
            encouragingMessages[
                Math.floor(Math.random() * encouragingMessages.length)
            ];

        return {
            level: "positive",
            title: "Keep supporting your well-being",
            message,
            primaryAction: null,
            urgent: false
        };
    }

    if (score >= 28) {
        return {
            level: "reduced",
            title: "Consider talking with someone",
            message:
                "Your recent check-ins suggest reduced well-being. Try talk with Povi, connect with someone you trust, or contact university mental-health support for further guidance.",
            primaryAction: "talk-to-povi",
            secondaryAction: "view-university-support",
            urgent: false
        };
    }

    return {
        level: "very-low",
        title: "Professional support is recommended",
        message:
            "Your recent check-ins suggest very low well-being. Please consider contacting a qualified mental-health professional or university counselling service promptly.",
        primaryAction: "book-support",
        secondaryAction: "view-urgent-resources",
        urgent: true
    };
}

app.listen(PORT, () => {
    console.log(`Server running at http://localhost:${PORT}`);
});