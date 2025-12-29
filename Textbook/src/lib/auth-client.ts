import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
    // Updated to use the live deployed Auth Server
    baseURL: "https://auth765.vercel.app"
});