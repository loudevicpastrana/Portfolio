int myAtoi(char* s) {

    int i=0;
    int sign = 1; // positive
    int digit = 0;
    int result = 0;

    //While s[index] == ' ', increment index.
    while(s[i] ==  ' ')
    {
        i++;
    }
    if(s[i] == '-')
    {
        sign = -1;
        i++;
    }
    else if(s[i] == '+')
        i++;
    else
    {}
    
    while (s[i] >= '0' && s[i] <= '9') {
    // Inside the loop, s[index] is a digit
    digit = s[i] - '0';
    if (result > (INT_MAX - digit) / 10)
    {
        if(sign == 1)
         return INT_MAX; 
        else 
         return INT_MIN;
    }
   
    result = result * 10 + digit;
    i++;

    }
    if(sign == 1)
        return result;
    else
        return (-1*result);
}   