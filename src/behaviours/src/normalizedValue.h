float normalizedValue(float x)
{
	float temp = x * 100;
	int val = round(temp);
	
	if ((val % 25) > 13)
	{
		int n = (val/25) + 1;
		return n*0.25;
	}
	else
	{
		int n = (val/25);
		return n*0.25;
	}
}
