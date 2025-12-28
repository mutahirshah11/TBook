import { betterAuth } from "better-auth";
import { Pool } from "pg";
import dotenv from 'dotenv';

dotenv.config({ path: '../.env' });

if (!process.env.NEON_DATABASE_URL) {
    throw new Error("NEON_DATABASE_URL is not defined");
}

export const auth = betterAuth({
    database: new Pool({
        connectionString: process.env.NEON_DATABASE_URL,
    }),
    emailAndPassword: {
        enabled: true,
        requireEmailVerification: false,
    },
    session: {
        expiresIn: 7 * 24 * 60 * 60, // 7 days
    },
    account: {
        accountLinking: {
            enabled: true,
        }
    },
    secret: process.env.AUTH_SECRET || 'fallback-secret-for-development',
    plugins: [
        // Add any required plugins here
    ],
    trustedOrigins: ['http://localhost:3000', 'http://localhost:8000'],
});
