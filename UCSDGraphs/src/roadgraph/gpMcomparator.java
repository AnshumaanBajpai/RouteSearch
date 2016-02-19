package roadgraph;
import java.util.Comparator;
public class gpMcomparator implements Comparator<gpModified> {
	@Override
	public int compare(gpModified gp1, gpModified gp2) {
		// Assume neither string is null. Real code should
		// probably be more robust
		// You could also just return x.length() - y.length(),
		// which would be more efficient.
		if (gp1.getDFS() < gp2.getDFS()) {
			return -1;
		}
		if (gp1.getDFS() > gp2.getDFS()) {
			return 1;
		}
		return 0;
	}

}
