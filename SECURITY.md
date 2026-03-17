# SECURITY.md

## 1. Scope

- This policy applies to a single Raspberry Pi used as a proof-of-concept with local sensors.
- The Pi is not connected to any network (no Wi‑Fi, no Ethernet, no Bluetooth).
- Physical access is the main security risk.

## 2. Proof-of-Concept Nature

- This setup is temporary and for experimentation and demos only.
- I do not store long-term or highly sensitive personal data on this Pi.
- I accept higher risk than production, but I avoid bad habits (e.g., reusing passwords, hard-coding real secrets).
- When the proof of concept ends, I will export needed data and wipe or re-image the SD card.

## 3. Physical Security

- I keep the Pi and sensors in a place where only I (or trusted people) can reach them.
- I secure cables and sensors so tampering is noticeable.
- If I lend or relocate the Pi, I treat it as untrusted when it returns and re-image the SD card if I have doubts.

## 4. Operating System and Storage

- I use an official Raspberry Pi OS image from a trusted source.
- If I suspect corruption or tampering, I re-image the SD card instead of trying to “fix” it in place.
- I keep a spare SD card with a known-good image and basic configuration for quick recovery.

## 5. User Accounts and Login

- I change any default usernames and passwords (including the default `pi` user, if present).
- I use a strong password for local console login, even though the Pi is offline.
- I disable or remove unnecessary services (e.g., SSH server) to avoid accidental future exposure if networking is enabled.

## 6. Sensor Data Handling

- I clearly separate raw sensor data logs from any other files (e.g., code, notes).
- I rotate or archive logs so the SD card does not fill up and cause failures.
- If sensor data could reveal private behavior (e.g., movement, presence), I treat those logs as sensitive and handle them carefully.

## 7. External Media and Code

- I only connect USB devices or SD cards that I trust and that I control.
- I avoid running scripts or binaries from unknown or unverified sources, even on this offline Pi.
- Before reusing storage from other projects, I fully format or re-image it.

## 8. Future Networking

- If I ever connect this Pi to a network, I will:
  - Enable a firewall and open only required ports.
  - Use strong authentication (keys or strong passwords) for any remote access.
  - Review and update this policy to cover new remote risks.
  - so until then stop being a pretentious fuck and just wait and coordinate.
c.
