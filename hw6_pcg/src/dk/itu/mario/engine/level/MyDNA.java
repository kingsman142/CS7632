package dk.itu.mario.engine.level;

import java.util.Random;
import java.util.*;

//Make any new member variables and functions you deem necessary.
//Make new constructors if necessary
//You must implement mutate() and crossover()


public class MyDNA extends DNA
{

	public int numGenes = 0; //number of genes
	private final String LANGUAGE = "abcdefghijklmnopqrstuvwxyz"; // our alphabet

	// Return a new DNA that differs from this one in a small way.
	// Do not change this DNA by side effect; copy it, change the copy, and return the copy.
	public MyDNA mutate ()
	{
		MyDNA copy = new MyDNA();
		//YOUR CODE GOES BELOW HERE
		Random r = new Random();
		String oldChromosome = this.getChromosome();

		// perform mutation
		int mutationIndex = r.nextInt(this.getLength()); // which character in the chromosome are we modifying?
		int mutationChar = LANGUAGE.charAt(r.nextInt(LANGUAGE.length())); // what are we replacing that old character with?
		String newChromosome = oldChromosome.substring(0, mutationIndex) + mutationChar + oldChromosome.substring(mutationIndex+1); // replace the old character
		assert newChromosome.length() == oldChromosome.length();
		copy.setChromosome(newChromosome);
		//YOUR CODE GOES ABOVE HERE
		return copy;
	}

	// Do not change this DNA by side effect
	public ArrayList<MyDNA> crossover (MyDNA mate)
	{
		ArrayList<MyDNA> offspring = new ArrayList<MyDNA>();
		//YOUR CODE GOES BELOW HERE
		int middleIndex = this.getLength() / 2;
		String ourChromosome = this.getChromosome();
		String mateChromosome = mate.getChromosome();

		// create + add first offspring
		String firstOffspringChromosome = mateChromosome.substring(0, middleIndex) + ourChromosome.substring(middleIndex);
		MyDNA firstOffspring = new MyDNA();
		firstOffspring.setChromosome(firstOffspringChromosome);
		offspring.add(firstOffspring);

		// create + add second offspring
		String secondOffspringChromosome = ourChromosome.substring(0, middleIndex) + mateChromosome.substring(middleIndex);
		MyDNA secondOffspring = new MyDNA();
		secondOffspring.setChromosome(secondOffspringChromosome);
		offspring.add(secondOffspring);
		//YOUR CODE GOES ABOVE HERE
		return offspring;
	}

	// Optional, modify this function if you use a means of calculating fitness other than using the fitness member variable.
	// Return 0 if this object has the same fitness as other.
	// Return -1 if this object has lower fitness than other.
	// Return +1 if this objet has greater fitness than other.
	public int compareTo(MyDNA other)
	{
		int result = super.compareTo(other);
		//YOUR CODE GOES BELOW HERE

		//YOUR CODE GOES ABOVE HERE
		return result;
	}


	// For debugging purposes (optional)
	public String toString ()
	{
		String s = super.toString();
		//YOUR CODE GOES BELOW HERE

		//YOUR CODE GOES ABOVE HERE
		return s;
	}

	public void setNumGenes (int n)
	{
		this.numGenes = n;
	}

}
