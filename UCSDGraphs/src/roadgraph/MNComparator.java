package roadgraph;

import java.util.Comparator;

public class MNComparator implements Comparator<MapNode> {
	@Override
	public int compare(MapNode node1, MapNode node2) {
		// Assume neither string is null. Real code should
		// probably be more robust
		// You could also just return x.length() - y.length(),
		// which would be more efficient.
		if (node1.getdistFstart() < node2.getdistFstart()) {
			return -1;
		}
		if (node1.getdistFstart() > node2.getdistFstart()) {
			return 1;
		}
		return 0;
	}
}