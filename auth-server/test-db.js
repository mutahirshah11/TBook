import { Pool } from 'pg';
import dotenv from 'dotenv';

dotenv.config({ path: '../.env' });

if (!process.env.DATABASE_URL) {
    console.error("DATABASE_URL is not defined");
    process.exit(1);
}

console.log("Testing database connection...");
console.log("DATABASE_URL:", process.env.DATABASE_URL.replace(/:[^@]*@/, ':***@'));

const pool = new Pool({
    connectionString: process.env.DATABASE_URL,
});

try {
    const client = await pool.connect();
    console.log("Database connection successful!");
    const result = await client.query('SELECT NOW()');
    console.log("Query result:", result.rows[0]);
    client.release();
} catch (error) {
    console.error("Database connection failed:", error.message);
} finally {
    await pool.end();
    console.log("Pool closed");
}