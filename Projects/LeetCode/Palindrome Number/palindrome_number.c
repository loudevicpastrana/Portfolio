bool isPalindrome(int x) {
    if (x < 0)
        return false;

    if (x < 10)
        return true;

    int arrNum[31];
    int i = 0, j = 0;
    int nTemp = x;

    // Extract digits into array
    while (nTemp > 0) {
        arrNum[i] = nTemp % 10;
        nTemp /= 10;
        i++;
    }

    // Compare digits from both ends
    i--; // Point to last digit
    while (i > j) {
        if (arrNum[i] != arrNum[j])
            return false;
        i--;
        j++;
    }

    return true;
}