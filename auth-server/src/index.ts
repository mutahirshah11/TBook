import dotenv from 'dotenv';
dotenv.config({ path: '../.env' });

import express from 'express';
import cors from 'cors';
import { auth } from './auth.js';
import { toNodeHandler } from 'better-auth/node';

const app = express();
const PORT = 4000;

// Enable CORS
app.use(cors({
    origin: ['http://localhost:3000', 'http://localhost:8000'],
    credentials: true,
    allowedHeaders: ['Content-Type', 'Authorization'],
}));

app.use(express.json());

console.log("Initializing Better Auth...");
const authHandler = toNodeHandler(auth);

app.get('/', (req, res) => {
    res.status(200).send('Auth Server is Running');
});

app.use('/api/auth', (req, res, next) => {
    authHandler(req, res).catch(err => {
        console.error("Better Auth Error:", err);
        next(err);
    });
});

if (process.env.NODE_ENV !== 'production') {
    const server = app.listen(PORT, () => {
        console.log(`Auth server running on http://localhost:${PORT}`);
        console.log("Ready to handle authentication requests");
    });

    server.on('error', (err) => {
        console.error("Server failed to start:", err);
    });
}

export default app;
