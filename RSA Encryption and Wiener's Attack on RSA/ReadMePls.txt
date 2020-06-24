RSA Encryption and Wiener's Attack on RSA - Python Implementation by Diana Doctor
This is my course project for Combinatorics and Cryptography. It is a Python implementation of the widely-used RSA (Rivest, Shamir, & Adleman) encryption algorithm which is one of the first public-key cryptosystems. The implementation includes a generation of a random prime key which is verified using the Miller-Rabin Primality Test. It also includes an implementation of the Wiener's Attack on RSA.

This program follows the order of function calls: 

5 Main Methods:
1) generate_keys() - call to generate the keys used for the encryption and decryption
2) encrypt_message() - call and enter a message to be encrypted
3) show_all_keys() - shows all private and public keys
4) decrypt_message() - call, enter the message to be decrypted along with the decryption key (d) which is included in the generated keys
5) attack_RSA() - tries to attack the RSA -- if the generated keys satisfies the Wiener's criteria (choose generate wiener keys option), the RSA can be attacked which means that the generated private keys can be computed

- The generate_keys() must be called first before encryption or decryption. New keys can be generated everytime this function is called.
- New messages can be encrypted and decrypted using the same keys just by calling again the encrypt_message() and decrypt_message() functions after the first run of the program
- The show_all_keys() function can be used to check the generated keys and can be used anytime

Useful References:
RSA
https://gist.github.com/JonCooperWorks/5314103

Attacks on RSA
https://github.com/LosFuzzys/rsa-attacks/tree/master/wiener
https://github.com/LosFuzzys/rsa-attacks/blob/master/wiener/attack.py
https://github.com/LosFuzzys/rsa-attacks/blob/master/wiener/Arithmetic.py
https://github.com/LosFuzzys/rsa-attacks/blob/master/wiener/ContinuedFractions.py
https://github.com/LosFuzzys/rsa-attacks/blob/master/wiener/README.md

Another
https://github.com/pablocelayes/rsa-wiener-attack

Vulnerabilities
Being:
M - plaintext message
N = p*q = primus number 1 * primus number 2
When encrypting with low encryption exponents (e.g.e=3) and small values of the M, (i.e. m < n^(1/e) ) the result of M^e is strictly less than the modulus n. In this case, ciphertexts can be easily decrypted by taking the th root of the ciphertext over the integers.
Because RSA encryption is a deterministic encryption algorithm (i.e., has no random component) an attacker can successfully launch a chosen plaintext attack against the cryptosystem, by encrypting likely plaintexts under the public key and test if they are equal to the ciphertext. A cryptosystem is called semantically secure if an attacker cannot distinguish two encryptions from each other even if the attacker knows (or has chosen) the corresponding plaintexts. As described above, RSA without padding (adding additional characters at the end) is not semantically secure.

Wiener’s attack:
Uses the continued fraction method to exploit a mistake made in the use of RSA. This error could be exploited when users are doing transactions using credit card or mobile devices such as phones. The public-key cryptosystem RSA is frequently used for security applications such as email, credit card payments, login network access and so on.

Example of Attack with Descriptions
https://sagi.io/2016/04/crypto-classics-wieners-rsa-attack/