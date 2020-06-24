# RSA Project for Cryptography and Combinatorics
# Diana Doctor (254765)

# Imported Libraries:
import random
import math
import copy
from fractions import gcd, Fraction
from decimal import *
from sympy import mod_inverse


# Functions to Generate Keys:
def generate_keys():
    #Generate public and private keys for RSA encryption.
    global n, e, d, blockSize
        
    key = input("Please enter 'w' for wiener and 'n' for normal keys: ")
    
    if key == 'w':
        wiener = True
        a = 10**20
        b = 10**21
    elif key == 'n':
        wiener = False
        a = 10**100
        b = 10**101
    
    #Set Parameters for the keys
    (n, e, d) = newKey(a, b, 50, wiener)
    blockSize = 15
    
    print("\nGenerated keys: \n\nn = {} \n\ne = {} \n\nd = {}".format(n,e,d,blockSize))

def newKey(a,b,t, wiener):
    global fn, p, q
    d = 0
    #Try to find two large pseudo primes p and q roughly between a and b.
    try:  
        #Find first large prime p
        p = findAPrime(a, b, t)
            
        while True:
            #Find second large prime q that is different from p
            q = findAPrime(a, b, t)
            
            #Check conditions for p and q
            if q != p:
                if wiener:
                    if (q < p and p < 2*q) or (p < q and q < 2*p):
                        break
                else:
                    break
                
    #Raise error if failed to find primes
    except:
        raise ValueError
    
    #Compute public key n
    n = p * q
    
    #Compute phi of n
    fn = (p-1) * (q-1)
    
    #Check if key has to satisfy Wiener Theorem
    if wiener:
        #Compute first for private key d (decryption key)
        while True:
            dr = random.uniform(1, (1/3 * pow(n,1/4))-1)
            
            if dr.is_integer():
                d = int(dr)
                
                #Check if d is invertible mod fn
                if math.gcd(d, fn) == 1:
                    #Compute for private key e (encryption key)
                    e = mod_inverse(int(d), fn)

                    #Such that e and fn are coprime
                    if math.gcd(int(e), fn) == 1:
                        break
    else:
        #Compute first for public key e (encryption key)
        while True:
            e = random.randint(1, fn)

            #Such that e and fn are coprime
            if math.gcd(e, fn) == 1:
                #Compute for private key d (decryption key)
                d = mod_inverse(e, fn)
                break
    
    assert (d*e) % fn == 1
    
    #Return generated keys
    return (n, e, d)

def findAPrime(a,b,t):
    #Return a prime number between a and b
    x = random.randint(a, b)
    
    #Look for random prime
    for i in range(0, int(10 * math.log(x) + 3)):
        
        #If satisfies Miller Rabin Primality Test then it is probably prime
        if millerRabin(x, t, False):
            return x
        
        #Keep searching for random prime
        else:
            x += 1
    
    #Raise ValueError if cannot find a random prime
    raise ValueError


# Functions for the Miller-Rabin Primality Test:
def extractKandM(n):
    #Extract two values k and m such that (2**k) * m = n-1 is satisfied
    n1 = n - 1
    assert n1 >= 0
    
    #Start with k = 0
    k = 0
    
    #Find k and m
    while True:
        #We divide n - 1 by 2**k until their quotient is odd = m
        m = n1 // pow(2,k)
        
        #If m is not divisible by 2 then it is odd
        if m % 2 != 0:
            break
        
        #Otherwise, increase k by 1
        k+=1

    #Return k and m
    return (k, m)

def millerRabin(n, t, show):
    #Miller Rabin pseudo-prime test
    #return True means likely a prime, (depending on k)
    #return False means definitely a composite.
    #Raise assertion error when n, t are not positive integers and n is not 1
    
    #Ensures that n is bigger than 1
    assert n >= 1
    
    #Ensures t is a positive integer since it will be the number of iterations
    assert t > 0
    
    #If n is 2 then it is surely prime
    if n == 2:
        if show:
            print("{} is 2 so it is surely prime!".format(n))
        return True

    #If n is divisible by 2 then it is surely not prime
    if n % 2 == 0:
        if show:
            print("{} is divisible by 2 so it is surely not prime!".format(n))
        return False
    
    #Extract values for k and m such that (2**k)*m = n-1
    extractKM = extractKandM(n)
    k = extractKM[0]
    m = extractKM[1]
    assert pow(2,k) * m == n - 1
    
    if show:
            print("Extracted values for k and m such that (2**k)*m = n - 1:\n")
            print("k: {}\n".format(k))
            print("m: {}\n".format(m))
    
    #Check n t-times for primality for different integers a
    for i in range (0, t):
        a = random.randint(2, n - 2)
        
        if show:
            print("\n***********************************\n")
            print("\nWe choose random integer a: {}".format(a))
            print("\nWe take gcd(n,a) = {}".format(math.gcd(n, a)))
            
        #If n is not coprime with random integer a then it is not prime
        if math.gcd(n, a) > 1:
            if show:
                print("\n***Greatest Common Divisor of n and a is not 1! n is not prime!***\n")
            return False

        if show:
            print("\nGreatest Common Divisor of n and a is 1!\n")
            
        if tryComposite(a, m, n, k, show) == False:
            if show:
                print("\n***{} is composite by Miller-Rabin Primality Test!***\n".format(n))
            return False

    # n is probably prime
    if show:
        print("***\n{} is probably prime by Miller-Rabin Primality Test!***\n".format(n))
    return True

#Inner process to check primality by Miller-Rabin 
def tryComposite(a, m, n, k, show):
    #Start with b = (a**m) mod n
    b = pow(a, m, n)
    
    if show:
        print("\nWe start with b0 = (a**m) mod n:\n")
        print("b0: {}\n".format(b))

    #If b = 1 or -1 mod n then n is composite with prob < (1/4)
    if b == 1 or b == n - 1:
        #We try other random integer a to check n for primality
        if show:
            print("b0 is equal to 1 or -1 mod n! n is probably prime!\n")
            print("\nWe check primality for other random integer a!\n")
        return None

    #Otherwise we compute bi from 1 to k-1
    else:

        #We start from b1 to bk-1
        for i in range (1,k):
            #We set bi = bi-1**2 mod n
            b = pow(b, 2, n)
            
            if show:
                print("\nWe proceed with b{} = (b{})**2 mod n:\n".format(i,i-1))
                print("b{}: {}\n".format(i,b))

            #If bi = 1 mod n then n is composite
            if b == 1:
                if show:
                    print("b{} is equal to 1 mod n!\n".format(i))
                return False

            #If bi = -1 mod n then n is composite with prob < (1/4)**i
            elif b == n - 1:
                #We try other random integer a to check n for primality
                if show:
                    print("b{} is equal to -1 mod n! n is probably prime!\n".format(i))
                    print("\nWe check primality for other random integer a!\n")
                return None

        #If at the end bk-1 != -1 mod n then n is composite
        if show:
            print("b{} is not equal to -1 mod n!\n".format(k-1))
            
        return False


# Function to Show All Keys
def show_all_keys():
    print("\nYour RSA Private/Public keys and Parameters:")
    print("\nn = {}". format(n))
    print("\np = {}". format(p))
    print("\nq = {}". format(q))
    print("\nfn = {}". format(fn))
    print("\ne = {}". format(e))
    print("\nd = {}". format(d))


# RSA Encryption Functions:
def encrypt_message():
    #Initialize global variables
    global cipherText, plainText, show_en
    
    #User input message to be encrypted
    message = input("\nPlease enter your message: ")
    
    show_pro = input("\nShow process for RSA encryption?(yes/no)").lower()

    if show_pro == 'yes':
        show_en = True
    else:
        show_en = False
    
    #Start Encryption
    #generate_keys()
    
    cipherText = []
    plainText = []
    
    #Convert letters to number
    numList = charToNumList(message)
        
    #Split the numbers to blocks
    numBlocks = numListToBlocks(numList)
       
    if show_en:
        print("\nMessage is transformed to blocks of numbers:")
        print("\nPlainText: {}".format(numBlocks))
        print("\nEncrypting each block by RSA: (m**e) mod n")
        
    #Encrypt number message for each block
    for blocks in numBlocks:
        plainText.append(blocks)
        
        #Follows m**e mod n
        cipherText.append(pow(blocks, e, n))
        
        if show_en:
            print("\n{} ---> {}".format(blocks, pow(blocks, e, n)))
            
    print("\nCipherText: {}".format(cipherText))
    
def charToNumList(message):
    #Converts string to list of integers based on ASCII values
    returnList = []
    
    #Turn each character to integer
    for chars in message:
        returnList.append(ord(chars))
        
    #Return list of converted message to integers
    return returnList
    
def numListToBlocks(numList):
    #Take a list of integers(each between 0 and 127), and combines them into block size
    #n using base 256. If len(numList) % blockSize != 0, use some random numbers to fill numList to make it
    returnList = []
    
    #Make copy of number list
    toProcess = copy.copy(numList)
    
    #Add random integers to follow the block size (padding)
    if len(toProcess) % blockSize != 0:
        for i in range (0, blockSize - len(toProcess) % blockSize):
            toProcess.append(random.randint(32, 126))
    
    #Create the list of blocks
    for i in range(0, len(toProcess), blockSize):
        block = 0
        
        #Enter the elements for each blocks bitwise left shift
        for j in range(0, blockSize):
            block += toProcess[i + j] << (8 * (blockSize - j - 1))
            
        returnList.append(block)
    
    #Return blocklist
    return returnList

    
# RSA Decryption Functions:
def decrypt_message():
    global show_de
    
    decrypt = input("\nDecrypt a message?:(yes/no)")
    
    if decrypt == 'yes':
        #User input message to be decrypted
        cmessage = input("\nPlease enter ciphered message: ").replace("[","").replace("]","")
        dec = int(input("\nPlease enter decryption key (d): "))
        
        show_pro = input("\nShow process for RSA decryption?(yes/no)").lower()

        if show_pro == 'yes':
            show_de = True
        else:
            show_de = False
            
        #Format ciphered message and to put on array
        cArrMessage = []
        cArrMessage = cmessage.split(',')
        
        numBlocks = []
        numList = []
        
        #Decrypt by blocks
        if show_de:
            print("\nDecrypting by blocks: (m**d) mod n")
            
        for blocks in cArrMessage:
            #Follows m**d mod n
            numBlocks.append(pow(int(blocks), dec, n))
            
            if show_de:
                print("{} ---> {}".format(blocks, pow(int(blocks), dec, n)))
            
        #Stores blocks to numbers list
        numList = blocksToNumList(numBlocks)
        
        #Convert numbers to String
        message = numListToString(numList)
        
        if show_en:
            print("\nDecrypted message: {}".format(numBlocks))
            print("\nMessage is transformed to string: {}".format(message))
            
        #Show decrypted message
        print("\nDecrypted Message: \n")
        print(message)
    
def blocksToNumList(blocks):
    #Turn blocks to number list
    toProcess = copy.copy(blocks)
    returnList = []
    
    #Separate each blocks
    for numBlock in toProcess:
        inner = []
        
        #Separate numbers for each characters
        for i in range(0, blockSize):
            inner.append(numBlock % 256)
            numBlock >>= 8
            
        inner.reverse()
        
        #Add each numbers to list
        returnList.extend(inner)
        
    #Return numberList
    return returnList
    
def numListToString(numList):
    #Converts a list of integers to a string based on ASCII values
    returnList = []
    returnString = ''
    
    #Convert numbers to characters
    for nums in numList:
        returnString += chr(nums)
        
    #Return converted message
    return returnString


# Attack on RSA by the Wiener Theorem Functions:
def cf_expansion(e, n):
    global en
    enList = []
    
    #Initial expansion
    quot = e // n
    rem = e % n
    enList.append(quot)
    
    while rem != 0:
        e1, n = n, rem
        quot = e1 // n
        rem = e1 % n
        enList.append(quot)

    #Return list of coefficients from the expansion
    return enList

def approximateKandD():
    global cfList
    enList = cf_expansion(e, n)
    
    if show:
        print("\nContinued Fraction Expansion of e/n: \n")
        print(enList)
    
    cfList = []
    
    for i in range(1, len(enList)):
        #Set current position of integer
        continuedFraction = 0
        currentInt = enList[i]
        
        #Invert the current integer
        invCurrentInt = Fraction(1, currentInt)
        
        #Set it is as current value of the sum
        invFractionSum = invCurrentInt
        
        j = i - 1
        
        #Add to the sum the previous integers
        while j != 0:
            
            #Get current integer
            prevInt = enList[j]
            
            #Add the previous integer
            invFractionSum += prevInt

            #Invert current integer
            invFractionSum = Fraction(1, invFractionSum)

            j-=1
        
        #Finalize the sum
        continuedFraction = invFractionSum
        
        #Add sum to the approximations of k and d
        cfList.append(continuedFraction)
        
    return cfList

def attack_RSA(): 
    global show
    
    check = input("\nCheck if RSA can be attacked?(yes/no)").lower()
    
    if check == 'yes':
        #Check requirements to attack
        if (q < p and p >= 2*q) or (p < q and q >= 2*p):
            #Cannot be attacked
            print("\nAttack on RSA Failed: Lower Exponent Attack fails! p < q < 2p is not satisfied!") 
            return
        
        if d >= 1/3 * pow(n,1/4):
            #Cannot be attacked
            print("\nAttack on RSA Failed: Lower Exponent Attack fails! n**1/4 > 3*d is not satisfied!")
            return
        
        assert pow(n,1/4) > 3*d
        
        answer = input("\nRSA can be attacked! Try attacking?(yes/no)").lower()
        
        if answer == 'yes':
            
            show_pro = input("\nShow process?(yes/no)").lower()

            if show_pro == 'yes':
                show = True
            else:
                show = False
            
            #Call function for approximating k and d
            probableKandD = approximateKandD()
            
            if show:
                print("\nApproximations of k/d: \n")
                print(probableKandD)

            #Try each probable k and d to retrieve fn = (p-1)(q-1)
            for i in range(0, len(probableKandD)):
                pi = probableKandD[i].numerator
                qi = probableKandD[i].denominator
                
                #Since k/d should be greater than e/n
                if Fraction(pi,qi) > Fraction(e,n):
                    
                    #Assumed value for fn
                    afn = Fraction((e*qi)-1,pi)
                    
                    if afn.denominator == 1:
                        if show:
                            print("\nUsed pi/qi as approximate to k/d: *By Wiener's Theorem: k/d > e/n* \n")
                            print("pi:\n")
                            print(pi)
                            print("\nqi:\n")
                            print(qi)
                            print("\nPossible phi(n) computed by (e*qi)-1/pi: \n")
                            print(afn)                    
                        
                        #Solve by quadratic formula
                        solve_quadratic(afn)
                            
def solve_quadratic(afn):
    #Solve quadratic equation (x**2 - (n - fn + 1)x + n = 0)
    #Set values for a, b, c
    a = 1
    b = - n + afn - 1
    c = n

    #Determine p and q by quadratic formula
    dc = pow(b,2) - (4*a*c)
    
    if dc > 0:
        #Get the roots of the polynomial
        root1 = int(-b) - Decimal(int(dc)).sqrt()
        root2 = int(-b) + Decimal(int(dc)).sqrt()
    
    #To check is roots are integers
    if root1 > 0 and root2 > 0:  
        ap = Decimal(root1/(2*a))
        aq = Decimal(root2/(2*a))
        
        if show:
            print("\nRoots of (x**2 - (n - fn + 1)x + n = 0): \n")
            print("1st root:\n")
            print(ap)
            print("\n2nd root:\n")
            print(aq)

        if int(aq)*int(ap) == n:
            adec = mod_inverse(e, afn)
            
            print("\n***We have found the factorization of n!***")
            print("\nn = {}".format(n))
            print("\np = {}".format(ap))
            print("\nq = {}".format(aq))
            print("\nd = {}".format(adec))
            
        else:
            if show:
                print("\n***Approximate keys are wrong!***")

################################################################
# Order of Function Calls for First Run

# Generate Keys
generate_keys()

# Encrypts message
encrypt_message()

# Decrypts message
decrypt_message()

# Tries to attack the RSA Encryption
attack_RSA()

# Shows all public and private keys of the cryptosystem
show_all_keys()

