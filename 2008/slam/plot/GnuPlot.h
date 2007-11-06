
#if 0
class GNUplot {
public:
	GNUplot() throw(string);
	~GNUplot();
	void operator ()(const string& command);
	// send any command to gnuplot
protected:
	FILE *gnuplotpipe;
};

GNUplot::GNUplot() throw(string) {
	gnuplotpipe=popen("gnuplot","w");
	if (!gnuplotpipe) {
		throw("Gnuplot not found !");
	}
}

GNUplot::~GNUplot() {
	fprintf(gnuplotpipe,"exit\n");
	pclose(gnuplotpipe);
}

void GNUplot::operator() (const string& command) {
	fprintf(gnuplotpipe,"%s\n",command.c_str());
	fflush(gnuplotpipe);
};

	/*GNUplot plotter;
	plotter("plot sin(x)");
	cout << "Type a charater and press Enter to exit" << endl;
	int x;
	cin >> x;*/
#endif

ofstream commandFile;
	commandFile.open("test.p");
	commandFile << "plot 'test.dat' using 1:2 w linespoints\n" << "pause -1" << endl;
	commandFile.close();

	ofstream dataFile;
	dataFile.open("test.dat");
	dataFile << "#test.txt\n";
	for(int i = 1; i < n; i++) {
		dataFile << i << "\t" << sqrt( pow(data[i],2) + pow(data[n - i],2) ) << "\n";
	}
	dataFile.close();

	system("gnuplot test.p");

