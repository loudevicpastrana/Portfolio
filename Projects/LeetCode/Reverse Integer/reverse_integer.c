int reverse(int x){

    int rev =0, nLastDigit = 0;


    while(x!=0)
    {
        nLastDigit = x%10; // 2

        if(rev > INT_MAX / 10 || (rev == INT_MAX / 10 && nLastDigit > 7))
            return 0;

        if(rev < INT_MIN / 10 || (rev == INT_MIN / 10 && nLastDigit < -8))
            return 0;

        rev = rev * 10 + nLastDigit; // rev = 32
        x/=10; // x= 1
    }

  

    return rev;
}